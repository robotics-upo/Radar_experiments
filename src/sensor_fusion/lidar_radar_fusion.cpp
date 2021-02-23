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
class LidarRadarFuser
{

public:
    LidarRadarFuser(): spinner(2), pointclouds_subs_sync_(NULL)
    {
        //Lidar and Radar pointcloud sync subscriber
        pnh_.param("sychronization_margin", synch_margin_queue_,10);

        pc_sub_lidar_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(pnh_, "/lidar_points", 1));
        pc_sub_radar_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(pnh_, "/radar_points", 1));

        pointclouds_subs_sync_ = new message_filters::Synchronizer<LidarRadarSyncPolicy>(LidarRadarSyncPolicy(synch_margin_queue_), *pc_sub_lidar_, *pc_sub_radar_);
        pointclouds_subs_sync_->registerCallback(&LidarRadarFuser::pointCloudsCallback, this);

        //Publishers
        virtual_2d_pointcloud_pub_  = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("virtual_2d_pointcloud", 1);
        ransac_result_pub_          = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("ransac_result", 1);
        fused_points_cloud_pub_     = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("fused_ranges_cloud", 1);
        radar_points_cloud_pub_     = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("radar_ranges_cloud", 1);
        lidar_points_cloud_pub_     = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("lidar_ranges_cloud", 1);
        final_points_cloud_pub_     = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("final_result_cloud", 1);

        overlapping_dist_pub_       = pnh_.advertise<std_msgs::Float32>("overlapping_distance", 1);
        
        //Params....
        pnh_.param("yaw_tolerance", yaw_tolerance_, 0.1);
        pnh_.param("ransac_distance_threshold", ransac_distance_threshold_, 0.01);
        pnh_.param("ransac_iterations", ransac_iterations_, 5);

        pnh_.param("min_ransac_pointcloud_size", min_ransac_pointcloud_size_, 10);
        
        pnh_.param("beta", beta_, 0.2);
        if(beta_ < 0.0 || beta_ > 1.0) beta_ = 0.2; //Default in case you try to set a bad value
        
        pnh_.param("distance_threshold", d_f_, 0.5);
        if(d_f_ < 0.0 ) d_f_ *= -1;
        
        pnh_.param("time_tolerance", time_tolerance_, 1.0);
        if(time_tolerance_ < 0) time_tolerance_ *= -1;

        pnh_.param("radar_dev", radar_dev_, 0.03);
        pnh_.param("lidar_dev", lidar_dev_, 0.02);

    }
    void run()
    {
        spinner.start();
        ros::waitForShutdown();
    }
private:
    void processRadar(const pcl::PointCloud<pcl::PointXYZ> &points){
        
        std::vector<double> dist_vector;
        maximum_range_radar_measurement = 0;
        double range;
        for (auto &it : points){
            range = dist2Origin(it);
            dist_vector.push_back(range);
            if(range > maximum_range_radar_measurement) maximum_range_radar_measurement = range;
        }
        
        average_range_radar_scan_ = std::accumulate(dist_vector.begin(), dist_vector.end(), 0.0) / dist_vector.size();

        overlapping_scan_field_r_f_ = average_range_radar_scan_ + beta_ * (maximum_range_radar_measurement - average_range_radar_scan_);
        std_msgs::Float32 overlapping_dist;
        overlapping_dist.data = overlapping_scan_field_r_f_;
        overlapping_dist_pub_.publish(overlapping_dist);

    }
    
    void processLidar(const pcl::PointCloud<pcl::PointXYZ> &points, const std::string _frame_id)
    {
        virtual_2d_scan_cloud_ = extract2DVirtualScans(points, _frame_id);

        ransac_result_cloud_ = applyLineRANSAC(virtual_2d_scan_cloud_, _frame_id);
    }
    void pointCloudsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg,
                             const sensor_msgs::PointCloud2::ConstPtr& radar_msg)
    {   
        ros::Duration time_diff = lidar_msg->header.stamp - radar_msg->header.stamp;
        // std::cout<< "Time difference: " << time_diff.toSec() << std::endl;
        
        if(time_diff.toSec() > time_tolerance_){
            ROS_WARN("Skipping pair of lidar-radar clouds because they exceed time tolerance %f > %f", time_diff.toSec(), time_tolerance_);
            return;
        }
        pcl::fromROSMsg(*lidar_msg, last_lidar_cloud_);

        sensor_msgs::PointCloud2 radar_msg_lidar_frame;
        pcl_ros::transformPointCloud(lidar_msg->header.frame_id, *radar_msg, radar_msg_lidar_frame,tf_listener_);
        pcl::fromROSMsg(radar_msg_lidar_frame, last_radar_cloud_);
        
        processRadar(last_radar_cloud_);
        processLidar(last_lidar_cloud_, lidar_msg->header.frame_id);

        //To apply Eq2 of the paper (fuseRanges function) wee need to iterate the lidar and the radar clouds 
        //We need to search for points that verifies that |R_lidar - R_radar| < d_F
        //In case two range measurements verify this inequality, calculate R fusion and put it into the result fused cloud
        fused_scan_result_ = createFusedCloud(lidar_msg->header.frame_id);
    }
    pcl::PointCloud<pcl::PointXYZ> createFusedCloud(const std::string &frame_id){


        //It's more efficient to iterate over radar cloud as it has less points that the lidar cloud (? sure ?)
        std::vector<std::pair<double, double>> ranges_vec;
        for(auto &it: ransac_result_cloud_)
            ranges_vec.emplace_back(std::make_pair<double, double>(dist2Origin(it), pointYaw(it)));

        std::vector<std::pair<double, double>> fused_ranges;
        std::vector<std::pair<double, double>> lidar_ranges;
        std::vector<std::pair<double, double>> radar_ranges;

        double radar_point_range;        
        double fused_range;
        //TODO There are repeated points pushed inside the final container. FIX
        for(auto &it: last_radar_cloud_){
            radar_point_range = dist2Origin(it);
            for(auto &ranges_it: ranges_vec){
                fused_range = fuseRanges(radar_point_range, radar_dev_, ranges_it.first, lidar_dev_);
                //!Lidar insertion conditions
                if( std::fabs( ranges_it.first - radar_point_range ) > d_f_ ){
                    lidar_ranges.emplace_back(ranges_it);
                }
                if( ranges_it.first > fused_range && ranges_it.first < 120.0 ){ //120 == infinity
                    lidar_ranges.emplace_back(ranges_it);
                }
                if( ranges_it.first - radar_point_range > d_f_ && ranges_it.first < fused_range){
                    lidar_ranges.emplace_back(ranges_it);
                }
                //! Radar Insertion conditions
                if( ranges_it.first - radar_point_range < -1*d_f_){
                    std::pair<double, double> p(radar_point_range, pointYaw(it));
                    radar_ranges.emplace_back(p);
                }
                if( ranges_it.first >= 120.0 && radar_point_range < 120.0 ){ //120 == infinity
                    std::pair<double, double> p(radar_point_range, pointYaw(it));
                    radar_ranges.emplace_back(p);
                }
                //! Fused ranges insertion condition
                if( std::fabs(ranges_it.first - radar_point_range) < d_f_ ){
                    std::pair<double, double> p(fused_range, ranges_it.second);
                    fused_ranges.emplace_back(p);
                }                
            }
        }

        //Undo ranges calculation-> get x,y points coordinates

         //Create cloud from fused ranges
        pcl::PointCloud<pcl::PointXYZ> result_cloud;
        pcl::PointCloud<pcl::PointXYZ> fused_points_cloud;
        pcl::PointCloud<pcl::PointXYZ> radar_points_cloud;
        pcl::PointCloud<pcl::PointXYZ> lidar_points_cloud;
        result_cloud.header.frame_id = frame_id;     
        fused_points_cloud.header.frame_id = frame_id;
        radar_points_cloud.header.frame_id = frame_id;
        lidar_points_cloud.header.frame_id = frame_id;

        pcl::PointXYZ p;
        p.z = 0;
        //Undo fused ranges cloud
        for(auto &it: fused_ranges){
            p.x = it.first*cos(it.second);
            p.y = it.first*sin(it.second);
            fused_points_cloud.push_back(p);
            result_cloud.push_back(p);
        }
        fused_points_cloud_pub_.publish(fused_points_cloud);
        //Undo radar ranges cloud
        for(auto &it: radar_ranges){
            p.x = it.first*cos(it.second);
            p.y = it.first*sin(it.second);
            radar_points_cloud.push_back(p);
            result_cloud.push_back(p);
        }
        radar_points_cloud_pub_.publish(radar_points_cloud);

        //Undo lidar ranges cloud
        for(auto &it: lidar_ranges){
            p.x = it.first*cos(it.second);
            p.y = it.first*sin(it.second);
            lidar_points_cloud.push_back(p);
            result_cloud.push_back(p);
        }
        lidar_points_cloud_pub_.publish(lidar_points_cloud);
        std::cout <<"Radar contributions: " << radar_points_cloud.points.size()
                  <<" Lidar: " << lidar_points_cloud.points.size()  
                  <<" Fused: " << fused_points_cloud.points.size() << std::endl;
        //Put all three clouds together       
        final_points_cloud_pub_.publish(result_cloud);

        return result_cloud;
    }
    //According to the paper the sigma_R is proportional to 1/Pe with Pe received power. Pe should be proportional to 1/R2 ?
    double radarDev(const pcl::PointXYZ &point){
        // return radar_dev_prop_const_ * pow(dist2Origin(point),2);
        return radar_dev_;
    }
    //According to the paper the sigma_L is proportional to the range measurement of the lidar.
    // From os1_lidar sensor dataset: http://data.ouster.io/downloads/OS1-gen1-lidar-sensor-datasheet.pdf
    // 0.8 - 2 m: ± 3 cm
    // 2 - 20 m: ± 1.5 cm
    // 20 - 60 m ± 3 cm
    // >60 m: ± 10 cm
    double lidarDev(const pcl::PointXYZ &point){
        //double range = dist2Origin(point);
        /*if(range < 2.0){
            return 0.03;
        }else if(range < 20){
            return 0.015;
        }else if(range < 60){
            return 0.03;
        }else{
            return 0.1;
        }*/
        return lidar_dev_;
    }
    double dist2Origin(const pcl::PointXYZ &point)
    {
        return sqrtf(point.x * point.x + point.y * point.y);
    }
    double pointYaw(const pcl::PointXYZ &point)
    {
        if (point.y == 0.0 && point.x == 0)
            return 0;

        return atan2(point.y, point.x);
    }
    //! Equation 2 in the paper 
    double fuseRanges(const double &r_radar, const double &radar_dev, const double &r_lidar, const double &lidar_dev){
        return r_radar + pow(radar_dev,2) / (pow(lidar_dev,2) + pow(radar_dev, 2)) * (r_lidar - r_radar);
    }
    pcl::PointCloud<pcl::PointXYZ> extract2DVirtualScans(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &frame_id)
    {
        //The first step to create this virtual 2D scan is to project all 3D points onto the plane by setting the
        //z-coordinate to zero.
        pcl::PointCloud<pcl::PointXYZ> out;
        out.header.frame_id = frame_id;

        pcl::PointXYZ point;
        point.z = 0;
        double yaw = 0;
        std::vector<pcl::PointXYZ> last_points;

        //A virtual 2D scan that contains primarily walls  can  thereafter  be  assembled  by  taking  one  point  out
        //of  each vertical raw scan (resp. two points for a yawing scan top). This point is chosen to be the one with
        //the largest distant to the center  of  the  robot.
        for (auto &it : cloud)
        {
            point.x = it.x;
            point.y = it.y;

            if (last_points.empty())
            {
                // yaw = atan2(it.y, it.x); //between [-pi,pi]
                last_points.push_back(point);
            }
            else if (pointYaw(it) < pointYaw(last_points.at(0)) + yaw_tolerance_ &&
                     pointYaw(it) > pointYaw(last_points.at(0)) - yaw_tolerance_) //If it fits in the margin
            {
                last_points.push_back(point);
            }
            else
            {

                pcl::PointXYZ final_point;
                for (auto &it : last_points)
                {
                    if (dist2Origin(it) > dist2Origin(final_point))
                        final_point = it;
                }
                last_points.clear();
                out.push_back(final_point);
                // std::cout << "[" << it.x << ", " << it.y << ", "<< yaw << "]"  << std::endl;
            }
        }
        virtual_2d_pointcloud_pub_.publish(out);

        return out;
    }
    pcl::PointCloud<pcl::PointXYZ> applyLineRANSAC(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &frame_id)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_intermediate(new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l;
        pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr ransac;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        *in_intermediate = cloud;

        for (int i = 0; i < ransac_iterations_; ++i)
        {
            if(in_intermediate->points.size() < min_ransac_pointcloud_size_ ) break; //It should be nice here to check the standard deviation 
                                                                                     // of remaining points to see how "spreaded" they are
                                                                                     // and continue iterating if they not much spreaded?

            model_l.reset(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(in_intermediate));
            ransac.reset(new pcl::RandomSampleConsensus<pcl::PointXYZ>(model_l));
            inliers.reset(new pcl::PointIndices());

            ransac->setDistanceThreshold(ransac_distance_threshold_);
            ransac->computeModel();
            ransac->getInliers(inliers->indices);
            for(auto &it: inliers->indices){
                out_cloud->points.push_back(in_intermediate->points[it]);
            }
            // pcl::copyPointCloud(*in_intermediate, inliers->indices, *out_cloud);

            extract.setInputCloud(in_intermediate);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*in_intermediate);
        }
        
        out_cloud->header.frame_id = frame_id;
        ransac_result_pub_.publish(out_cloud);

        return *out_cloud;
    }
   
    
    //
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> LidarRadarSyncPolicy; 
    
    message_filters::Synchronizer<LidarRadarSyncPolicy> *pointclouds_subs_sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub_lidar_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub_radar_;
    
    ros::NodeHandle pnh_{"~"};
    ros::AsyncSpinner spinner;
    ros::Publisher  virtual_2d_pointcloud_pub_;
    ros::Publisher  ransac_result_pub_;
    ros::Publisher  overlapping_dist_pub_;
    ros::Publisher  fused_points_cloud_pub_;
    ros::Publisher  radar_points_cloud_pub_;
    ros::Publisher  lidar_points_cloud_pub_;
    ros::Publisher  final_points_cloud_pub_;
    tf::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZ> last_radar_cloud_;
    pcl::PointCloud<pcl::PointXYZ> last_lidar_cloud_;
    pcl::PointCloud<pcl::PointXYZ> virtual_2d_scan_cloud_;
    pcl::PointCloud<pcl::PointXYZ> ransac_result_cloud_;
    //! Cloud to store final result
    pcl::PointCloud<pcl::PointXYZ> fused_scan_result_;

    //!Calculated params
    double overlapping_scan_field_r_f_;//Calculated at every fusion cycle
    double average_range_radar_scan_;
    double maximum_range_radar_measurement;

    double radar_dev_;
    double lidar_dev_;

    //! Node Params
    int synch_margin_queue_;
    double time_tolerance_;     

    //! Algorithm params
    double yaw_tolerance_; // Used to extract virtual 2d scans. If two points have a yaw
                           // difference less than yaw_tolerance, we assume that they lie on the same
                           // vertical linea

    double ransac_distance_threshold_;
    int ransac_iterations_;
    int min_ransac_pointcloud_size_;

    double beta_;
    double d_f_;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_radar_sensor_fuser");

    LidarRadarFuser fuser;

    fuser.run();

    return 0;
}