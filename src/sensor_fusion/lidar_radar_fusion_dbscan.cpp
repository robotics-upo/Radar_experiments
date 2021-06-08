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

#include "math_utils.hpp"

// #define STEP_BY_STEP
class LidarRadarFuser
{
    using DoublePair = std::pair<double, double>;
public:
    LidarRadarFuser(): spinner(2), pointclouds_subs_sync_(NULL)
    {
        //Lidar and Radar pointcloud sync subscriber
        pnh_.param("sychronization_margin", synch_margin_queue_,10);

        pc_sub_lidar_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(pnh_, "/lidar_scan", 1));
        pc_sub_radar_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(pnh_, "/radar_scan", 1));

        pointclouds_subs_sync_ = new message_filters::Synchronizer<LidarRadarSyncPolicy>(LidarRadarSyncPolicy(synch_margin_queue_), *pc_sub_lidar_, *pc_sub_radar_);
        pointclouds_subs_sync_->registerCallback(&LidarRadarFuser::pointCloudsCallback, this);

        fused_scan_pub_ = pnh_.advertise<sensor_msgs::LaserScan>("fused_scan", 2);

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


        pnh_.param("lidar_max_range", lidar_max_range_, 120.0);
        if(lidar_max_range_ < 0) lidar_max_range_ *= -1;
        //Radar enhacement params
        pnh_.param("max_radar_acc_yaw_incr", max_radar_acc_yaw_incr_, 0.4);
        pnh_.param("max_radar_acc_time", max_radar_acc_time_, 1.0);
        pnh_.param("odom_frame_id", odom_frame_id_, (std::string)"odom");
        pnh_.param("robot_base_frame_id", robot_base_frame_id_, (std::string)"base_link");

        double yaw_res_deg, azim_res_deg;
        pnh_.param("inflate_radar_points_with_resolution", inflate_radar_points_from_angular_resolution_, false);
        pnh_.param("angular_resolution_yaw", yaw_res_deg, 10.0);
        pnh_.param("angular_resolution_azimuth", azim_res_deg, 10.0);
        pnh_.param("point_step", point_step_, 0.03);
        radar_angular_resolution_azimuth_ = azim_res_deg / 180 * M_PI;
        radar_angular_resolution_yaw_     = yaw_res_deg  / 180 * M_PI;

        std::cout << "\033[1;31m\n---------------------------------------------\033[0m" << std::endl 
                  << "Lidar - Radar Scans Fuser Params:    " <<std::endl << std::endl
                  << "\tSynchronization Margin:            " << synch_margin_queue_ << std::endl
                  << "\tYaw Tolerance (rad):               " << yaw_tolerance_  << std::endl
                  << "\tBeta:                              " << beta_ << std::endl
                  << "\tDistance Threshold (d_F):          " << d_f_ << std::endl
                  << "\tTime Tolerance (s):                " << time_tolerance_ << std::endl
                  << "\tLidar Std Dev (m):                 " << lidar_dev_ << std::endl
                  << "\tRadar Std Dev (m):                 " << radar_dev_ << std::endl
                  << "\tInflate Radar Using Ang. Res:      " << inflate_radar_points_from_angular_resolution_ << std::endl
                  << "\tAngular Resolution Yaw (deg):      " << yaw_res_deg << std::endl
                  << "\tAngular Resolution Azimuth (deg):  " << azim_res_deg << std::endl
                  << "\tAngular inflation points step(m):  " << point_step_ << std::endl
                  << "\tBase Frame ID:                     " << robot_base_frame_id_ << std::endl
                  << "\tOdom Frame ID:                     " << odom_frame_id_ << std::endl
                  << "\033[1;31m---------------------------------------------\033[0m" << std::endl;
        configLineMarker(line_markers_);
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
    
    void pointCloudsCallback(const sensor_msgs::LaserScan::ConstPtr& lidar_msg,
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
    }
    //Return pairs of range, theta points that verifies that the yaw 
    template <typename T>
    DoublePair searchPointWithYaw(const pcl::PointCloud<T> &_points, const double &_yaw, const double &_tolerance){
        
        std::vector<double> r_res, r_yaw;
        DoublePair p;
        for(const auto &it: _points){
            p = xyToPolar(it);
            if( p.second < _yaw + _tolerance &&
                p.second > _yaw - _tolerance ){
                    r_res.push_back(p.first);
                    r_yaw.push_back(p.second);
                }
        }
        p.first = p.second = 0;
        if(! r_res.empty() ){
            p.first  = std::accumulate(r_res.begin(), r_res.end(), 0.0) / r_res.size();
            p.second = std::accumulate(r_yaw.begin(), r_yaw.end(), 0.0) / r_yaw.size();
        }

        return p;
    }
    sensor_msgs::LaserScan createFusedScan(const sensor_msgs::LaserScan &lidar_scan, const sensor_msgs::LaserScan &radar_scan){
        sensor_msgs::LaserScan ret = radar_scan;

        // First of all, we consider that the radar points are OK and try to merge with radar info
        int cont = 0;
        int n_radar = 0;
        int fused_cont = 0;
        auto points = rangesToPoints(lidar_scan);
        DBSCAN dbscan(5, 0.5, points);
        bool use_dbscan = true;
        int n_clusters = dbscan.run();
        ROS_INFO("DBSCAN run. N clusters: %d", n_clusters);
        for (cont = 0; cont < lidar_scan.ranges.size(); cont++) {
            auto range = ret.ranges[cont];

            
            if (range < lidar_max_range_) {
                if (fabs(lidar_scan.ranges[cont] - range) < d_f_ ) {
                    range = fuseRanges(range, radarDev(), lidar_scan.ranges[cont], lidarDev(lidar_scan.ranges[cont]));
                }
                ret.ranges[cont] = range;

                n_radar ++; fused_cont++;
                // Then add the lidar points that likely also are true: expand the laser in the proximities of the radar measures:
                int n_consider = ceil(yaw_tolerance_ / lidar_scan.angle_increment);
                // ROS_INFO("N_consider: %d", n_consider);

                bool consider_left = true;
                bool consider_right = true;
                double prev_range_left = range;
                double prev_range_right = range;
                int max_fused_cont = 0;

                if (!use_dbscan) {

                    for (int i = 1; i < n_consider && (consider_right || consider_left); i++) { //
                        int a = cont - i;

                        if (a >= 0 && consider_left) {
                            if ( fabs(prev_range_left - lidar_scan.ranges[a])/prev_range_left < 0.1 ) {
                                ret.ranges[a] = lidar_scan.ranges[a];
                                prev_range_left = lidar_scan.ranges[a];
                                fused_cont++;
                            } else {
                                consider_left = false;
                            }
                        }
                        a = cont + i;
                        if (a < ret.ranges.size() && consider_right) {
                            if ( fabs(prev_range_right - lidar_scan.ranges[a])/prev_range_right < 0.1 ) {
                                ret.ranges[a] = lidar_scan.ranges[a];
                                prev_range_right = lidar_scan.ranges[a];
                                fused_cont++;
                                max_fused_cont = fused_cont;
                            } else {
                                consider_right = false;
                            }
                        } 

                    }
                    cont += max_fused_cont;
                } else {
                    int cluster = -3;
                    points = dbscan.getPoints();
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
        }


    
        
        std::cout <<" Radar scan points: " << n_radar
                  <<" Lidar scan points: " << lidar_scan.ranges.size()  
                  <<" Fused scan points: " << fused_cont << std::endl;

        return ret;
    }
    //According to the paper the sigma_R is proportional to 1/Pe with Pe received power. Pe should be proportional to 1/R2 ?
    template <typename T>
    DoublePair xyToPolar(const T &point){
        return std::make_pair<double, double>(dist2Origin(point), pointYaw(point));
    }
    template <typename T>
    void fromPolarToXY(const DoublePair  &point , T &xy_point){
        xy_point.x = point.first*cos(point.second);
        xy_point.y = point.first*sin(point.second);
    }
    template <typename T=pcl::PointXYZ>
    T fromPolarToXY(const DoublePair &point){
        T p;
        p.x = point.first*cos(point.second);
        p.y = point.first*sin(point.second);
        return p;
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

    void configLineMarker(visualization_msgs::Marker &_marker){
        _marker.header.frame_id = robot_base_frame_id_;
        _marker.header.stamp = ros::Time::now();
        _marker.ns = "sensor_fusion";
        _marker.id = 0;
        _marker.type = visualization_msgs::Marker::LINE_LIST;
        _marker.action = visualization_msgs::Marker::ADD;
        _marker.pose.position.x = 1;
        _marker.pose.position.y = 1;
        _marker.pose.position.z = 1;
        _marker.pose.orientation.x = 0.0;
        _marker.pose.orientation.y = 0.0;
        _marker.pose.orientation.z = 0.0;
        _marker.pose.orientation.w = 1.0;
        _marker.scale.x = 0.1;
        _marker.color.a = 1.0; // Don't forget to set the alpha!
        _marker.color.r = 0.0;
        _marker.color.g = 1.0;
        _marker.color.b = 1.0;
    }
    //
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> LidarRadarSyncPolicy; 
    
    message_filters::Synchronizer<LidarRadarSyncPolicy> *pointclouds_subs_sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> pc_sub_lidar_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> pc_sub_radar_;
    
    ros::NodeHandle pnh_{"~"};
    ros::AsyncSpinner spinner;
    ros::Subscriber odom_sub_;
    ros::Publisher  fused_scan_pub_;
    ros::Publisher  ransac_lines_pub_;
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

    double max_radar_acc_time_;
    double max_radar_acc_yaw_incr_;

    std::ofstream radar_data_file_;
    bool save_radar_data_{false};

    bool inflate_radar_points_from_angular_resolution_;
    double radar_angular_resolution_yaw_;
    double radar_angular_resolution_azimuth_;
    double point_step_;
    std::string radar_original_frame_;

    visualization_msgs::Marker line_markers_;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_radar_sensor_fuser_gm");

    LidarRadarFuser fuser;

    fuser.run();

    return 0;
}