//! Basic fusion based on gaussian mixture of Radar measures

#include <ros/ros.h>
#include <math.h>
#include <numeric>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <fstream>

#include "dbscan/dbscan.h"
#include "dbscan/dbscan_lines.h"

#include "math_utils.hpp"

std_msgs::ColorRGBA getColor(int i);

class LidarRadarFuser
{
    using DoublePair = std::pair<double, double>;
public:
    LidarRadarFuser(): spinner(2), laser_scans_subs_sync_(NULL), radar_data_file_("radar_stats.m", std::ios::out)
    {
        //Lidar and Radar pointcloud sync subscriber
        pnh_.param("sychronization_margin", synch_margin_queue_,10);

        ls_sub_lidar_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(pnh_, "/lidar_scan", 1));
        ls_sub_radar_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(pnh_, "/radar_scan", 1));

        laser_scans_subs_sync_ = new message_filters::Synchronizer<LidarRadarSyncPolicy>(LidarRadarSyncPolicy(synch_margin_queue_), *ls_sub_lidar_, *ls_sub_radar_);
        laser_scans_subs_sync_->registerCallback(&LidarRadarFuser::laser_scansCallback, this);

        fused_scan_pub_ = pnh_.advertise<sensor_msgs::LaserScan>("fused_scan", 2);
        segmented_scan_pub_ = pnh_.advertise<sensor_msgs::LaserScan>("segmented_scan", 2);
        fused_scan_marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("fused_scan_marker", 2);

        //Params....
        pnh_.param("yaw_tolerance", yaw_tolerance_, 0.1);

        pnh_.param("beta", beta_, 0.2);
        if(beta_ < 0.0 || beta_ > 1.0) beta_ = 0.2; //Default in case you try to set a bad value

        pnh_.param("distance_threshold", d_f_, 0.3);
        if(d_f_ < 0.0 ) d_f_ *= -1;

        pnh_.param("time_tolerance", time_tolerance_, 1.0);
        if(time_tolerance_ < 0) time_tolerance_ *= -1;

        pnh_.param("radar_dev", radar_dev_, 0.03);
        pnh_.param("lidar_dev", lidar_dev_, 0.02);

        pnh_.param("use_dbscan_lines", m_use_dbscan_lines, true);
        pnh_.param("lidar_max_range", lidar_max_range_, 120.0);
        if(lidar_max_range_ < 0) lidar_max_range_ *= -1;
        //Radar enhacement params
        pnh_.param("odom_frame_id", odom_frame_id_, (std::string)"odom");
        pnh_.param("robot_base_frame_id", robot_base_frame_id_, (std::string)"base_link");

        std::cout << "\033[1;31m\n---------------------------------------------\033[0m" << std::endl
                  << "Lidar - Radar Scans Fuser Params:    " <<std::endl << std::endl
                  << "\tSynchronization Margin:            " << synch_margin_queue_ << std::endl
                  << "\tYaw Tolerance (rad):               " << yaw_tolerance_  << std::endl
                  << "\tBeta:                              " << beta_ << std::endl
                  << "\tDistance Threshold (d_F):          " << d_f_ << std::endl
                  << "\tTime Tolerance (s):                " << time_tolerance_ << std::endl
                  << "\tLidar Std Dev (m):                 " << lidar_dev_ << std::endl
                  << "\tRadar Std Dev (m):                 " << radar_dev_ << std::endl
                  << "\tBase Frame ID:                     " << robot_base_frame_id_ << std::endl
                  << "\tOdom Frame ID:                     " << odom_frame_id_ << std::endl
                  << "\033[1;31m---------------------------------------------\033[0m" << std::endl;
        
    }
    ~LidarRadarFuser(){
        if(save_radar_data_){
            std::cout << "Closing file" << std::endl;
            radar_data_file_.close();
        }
    }
    void run()
    {
        spinner.start();
        ros::waitForShutdown();
    }
private:

    std::vector<Point> rangesToPoints(const sensor_msgs::LaserScan &scan) {
        std::vector<Point> ret;

        double a = scan.angle_min;
        for (int i = 0; i < scan.ranges.size(); i++, a += scan.angle_increment) {
            double r = scan.ranges[i];
            Point p;
            p.x = r * cos(a);
            p.y = r * sin(a);
            p.z = 0;
            p.clusterID = UNCLASSIFIED;
            p.original_id = i;

            ret.push_back(p);
        }
        return ret;
    }

    void laser_scansCallback(const sensor_msgs::LaserScan::ConstPtr& lidar_msg,
                             const sensor_msgs::LaserScan::ConstPtr& radar_msg)
    {
        // We will take as inputs laser scans obtained from the PC
        ros::Duration time_diff = lidar_msg->header.stamp - radar_msg->header.stamp;

        radar_original_frame_ = radar_msg->header.frame_id;

        if(time_diff.toSec() > time_tolerance_){
            ROS_WARN("Skipping pair of lidar-radar clouds because they exceed time tolerance %f > %f", time_diff.toSec(), time_tolerance_);
            return;
        }

        //To apply Eq2 of the paper (fuseRanges function) wee need to iterate the lidar and the radar clouds
        //We need to search for points that verifies that |R_lidar - R_radar| < d_F
        //In case two range measurements verify this inequality, calculate R fusion and put it into the result fused cloud
        auto fused_scan = createFusedScan(*lidar_msg, *radar_msg);
        fused_scan_pub_.publish (fused_scan);

        auto segmented_scan = createSegmentedScan(*lidar_msg);
        segmented_scan_pub_.publish(segmented_scan);
    }

    sensor_msgs::LaserScan createSegmentedScan(const sensor_msgs::LaserScan &lidar_scan){
        auto points = rangesToPoints(lidar_scan);
        DBSCANLines dbscan(5, 0.5, points);
        sensor_msgs::LaserScan ret = lidar_scan;
        dbscan.run();
        auto clustered_points = dbscan.getPoints();

        // ROS_INFO("NLines = ", dbscan.m_n_lines);

        for (auto p:clustered_points) {
            if (p.clusterID < 0 || p.clusterID >= dbscan.m_n_lines) {
                ret.ranges[p.original_id] = std::numeric_limits<float>::infinity();
            }
        }

        return ret;
    }

    sensor_msgs::LaserScan createFusedScan(const sensor_msgs::LaserScan &lidar_scan, const sensor_msgs::LaserScan &radar_scan){
        sensor_msgs::LaserScan ret = radar_scan;

        // First of all, we consider that the radar points are OK and try to merge with radar info
        int cont = 0;
        int n_radar = 0;
        int fused_cont = 0;
        auto points = rangesToPoints(lidar_scan);
        DBSCAN *dbscan;

        if (!m_use_dbscan_lines)
            dbscan = new DBSCAN(5, 0.5, points);
        else
            dbscan = new DBSCANLines(5, 0.5, points);
        bool use_dbscan = true;
        int n_clusters = dbscan->run();
        ROS_INFO("DBSCAN run. Points: %d. N clusters: %d", (int)points.size(), n_clusters);

        // Publish marker
        fused_scan_marker_pub_.publish(pointsToMarker(dbscan->getPoints(), lidar_scan.header.frame_id));

        for (cont = 0; cont < lidar_scan.ranges.size(); cont++) {
            auto range = ret.ranges[cont];

            if (range < lidar_max_range_) {
                if (fabs(lidar_scan.ranges[cont] - range) < d_f_ ) {
                    range = fuseRanges(range, radarDev(), lidar_scan.ranges[cont], lidarDev(lidar_scan.ranges[cont]));
                }
                ret.ranges[cont] = range;

                // Then add the lidar points that likely also are true: expand the radar measure to the closest cluster
                int cluster = -3;
                points = dbscan->getPoints();
                for (auto p:points) {
                    if (p.original_id == cont) {
                        cluster = p.clusterID;
                    }
                }
                
                if (cluster >= 0) {
                    for (auto p:points) {
                        if (p.clusterID == cluster && p.original_id != cont) {
                            ret.ranges[p.original_id] = lidar_scan.ranges[p.original_id];
                        }
                    }
                }

            }
        }

        for (auto p:ret.ranges) {
            if (p < lidar_max_range_) {
                fused_cont++;
            }
        }

        for (auto p:radar_scan.ranges) {
            if (p < lidar_max_range_) {
                n_radar++;
            }
        }

        int n_lidar = 0;

        for (auto p:lidar_scan.ranges) {
            if (p < lidar_max_range_ && p > 0.02) {
                n_lidar++;
            }
        }

        int n_cluster = 0;
        int n_lines = 100000;
        auto db_line = dynamic_cast<DBSCANLines*>(dbscan);
        if (db_line != NULL) {
            n_lines = db_line->m_n_lines;
        }
        for (auto p:points) {
            if (p.clusterID >= 0 && p.clusterID < n_lines) {
                n_cluster++;
            }
        }

        radar_data_file_ << radar_scan.header.stamp << "\t";
        radar_data_file_ << n_lidar << " " << n_radar << "\t" << fused_cont << "\t" << n_cluster << "\n";

        std::cout <<" Radar scan points: " << n_radar
                  <<" Lidar scan points: " << n_lidar
                  <<" Clustered lidar scan points: " << fused_cont
                  <<" Fused scan points: " << fused_cont << " N_lines = " << n_lines << std::endl;

        delete dbscan;

        return ret;
    }

    double radarDev(){
        // return radar_dev_prop_const_ * pow(dist2Origin(point),2);
        return radar_dev_;
    }
    //According to the paper the sigma_L is proportional to the range measurement of the lidar.
    // From os1_lidar sensor dataset: http://data.ouster.io/downloads/OS1-gen1-lidar-sensor-datasheet.pdf
    // 0.8 - 2 m: ± 3 cm
    // 2 - 20 m: ± 1.5 cm
    // 20 - 60 m ± 3 cm
    // >60 m: ± 10 cm
    double lidarDev(const double &range){

        if(range < 2.0){
            return 0.03;
        }else if(range < 20){
            return 0.015;
        }else if(range < 60){
            return 0.03;
        }else{
            return 0.1;
        }
    }

    //! Equation 2 in the paper
    double fuseRanges(const double &r_radar, const double &radar_dev, const double &r_lidar, const double &lidar_dev){
        return r_radar + pow(radar_dev,2) / (pow(lidar_dev,2) + pow(radar_dev, 2)) * (r_lidar - r_radar);
    }

    visualization_msgs::Marker pointsToMarker(const std::vector<Point> points, const std::string frame_id){
        visualization_msgs::Marker _marker;
        _marker.header.frame_id = frame_id;
        _marker.header.stamp = ros::Time::now();
        _marker.ns = "sensor_fusion";
        _marker.id = 0;
        _marker.type = visualization_msgs::Marker::POINTS;
        _marker.action = visualization_msgs::Marker::ADD;
        _marker.pose.position.x = 0;
        _marker.pose.position.y = 0;
        _marker.pose.position.z = 0;
        _marker.pose.orientation.x = 0.0;
        _marker.pose.orientation.y = 0.0;
        _marker.pose.orientation.z = 0.0;
        _marker.pose.orientation.w = 1.0;
        _marker.scale.x = 0.15;
        _marker.scale.y = 0.15;
        _marker.scale.z = 0.15;
        _marker.color.a = 1.0; // Don't forget to set the alpha!
        _marker.color.r = 0.0;
        _marker.color.g = 1.0;
        _marker.color.b = 1.0;

        for (auto p:points) {
            if (fabs(p.x) > lidar_max_range_ || fabs(p.y) > lidar_max_range_ )
                continue;

            geometry_msgs::Point gp;
            
            gp.x = p.x;
            gp.y = p.y;
            gp.z = p.z;
            _marker.points.push_back(gp);
            _marker.colors.push_back(getColor(p.clusterID));
        }

        return _marker;
    }
    //
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> LidarRadarSyncPolicy;

    message_filters::Synchronizer<LidarRadarSyncPolicy> *laser_scans_subs_sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> ls_sub_lidar_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> ls_sub_radar_;

    ros::NodeHandle pnh_{"~"};
    ros::AsyncSpinner spinner;
    ros::Subscriber odom_sub_;
    ros::Publisher  fused_scan_pub_;
    ros::Publisher  segmented_scan_pub_;
    ros::Publisher  fused_scan_marker_pub_;
    tf::TransformListener tf_listener_;

    bool first{true}; //Used to detect the init of the server, when it is initialized it calls the callback at the startup and overrides the launch params(whe dont want this)


    std::string odom_frame_id_, robot_base_frame_id_;
    //!Calculated params
    double overlapping_scan_field_r_f_;//Calculated at every fusion cycle
    //! Standard deviation of sensors
    double radar_dev_;
    double lidar_dev_;

    //! Node Params
    int synch_margin_queue_;
    double time_tolerance_;

    //! Algorithm params
    double yaw_tolerance_; // Used to extract virtual 2d scans. If two points have a yaw
                           // difference less than yaw_tolerance, we assume that they lie on the same
                           // vertical linea

    double ransac_distance_threshold_lidar_, ransac_distance_threshold_radar_;
    int ransac_iterations_lidar_, ransac_iterations_radar_;
    int min_ransac_pointcloud_size_lidar_,min_ransac_pointcloud_size_radar_;

    double beta_;
    double d_f_;

    double lidar_max_range_;

    

    std::ofstream radar_data_file_;
    bool save_radar_data_{false};
    bool m_use_dbscan_lines;

    std::string radar_original_frame_;

};

std_msgs::ColorRGBA getColor(int i) {
  // Different colors for planes
  i = i % 6;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  switch (i) {
      case 0:
        color.b = 1.0;
        break;

      case 1:
        color.g = 1.0;
        break;

      case 2:
        color.r = 1.0;
        break;

      case 3:
        color.r = 1.0;
        color.b = 1.0;
        break;

      case 4:
        color.g = 1.0;
        color.b = 1.0;
        break;

      case 5:
        color.g = 1.0;
        color.r = 1.0;
        break;
  }


  return color;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_radar_sensor_fuser_gm");

    LidarRadarFuser fuser;

    fuser.run();

    return 0;
}