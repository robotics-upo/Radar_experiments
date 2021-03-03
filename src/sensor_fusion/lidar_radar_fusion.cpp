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
#include <dynamic_reconfigure/server.h>
#include <radar_experiments/LidarFusionConfig.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <fstream>

// #define STEP_BY_STEP
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
        lidar_ransac_result_pub_    = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("lidar_ransac_result", 1);
        radar_ransac_result_pub_    = pnh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("radar_ransac_result", 1);
        fused_points_cloud_pub_     = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("fused_ranges_cloud", 1);
        radar_points_cloud_pub_     = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("radar_ranges_cloud", 1);
        lidar_points_cloud_pub_     = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("lidar_ranges_cloud", 1);
        final_points_cloud_pub_     = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("final_result_cloud", 1);
        accumulated_radar_cloud_pub_= pnh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("accumulated_radar_cloud", 1);
        inflated_radar_cloud_pub_   = pnh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("inflated_radar_cloud", 1);

        overlapping_dist_pub_       = pnh_.advertise<std_msgs::Float32>("overlapping_distance", 1);
        ransac_lines_pub_           = pnh_.advertise<visualization_msgs::Marker>("ransac_line_markers", 1);
        
        dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<radar_experiments::LidarFusionConfig>);
        dynamic_recong_cb_f_.reset(new   dynamic_reconfigure::Server<radar_experiments::LidarFusionConfig>::CallbackType);

        *dynamic_recong_cb_f_ = boost::bind(&LidarRadarFuser::dynamicReconfigureCallback, this, _1, _2);
        dynamic_reconf_server_->setCallback(*dynamic_recong_cb_f_);

        odom_sub_ = pnh_.subscribe("/odom", 1, &LidarRadarFuser::odomCallback, this);
        //Params....
        pnh_.param("yaw_tolerance", yaw_tolerance_, 0.1);

        pnh_.param("lidar_ransac_distance_threshold", ransac_distance_threshold_lidar_, 0.01);
        pnh_.param("lidar_ransac_iterations", ransac_iterations_lidar_, 5);
        pnh_.param("lidar_min_ransac_pointcloud_size", min_ransac_pointcloud_size_lidar_, 10);
        
        pnh_.param("radar_ransac_distance_threshold", ransac_distance_threshold_radar_, 0.01);
        pnh_.param("radar_ransac_iterations", ransac_iterations_radar_, 5);
        pnh_.param("radar_min_ransac_pointcloud_size", min_ransac_pointcloud_size_radar_, 10);

        pnh_.param("beta", beta_, 0.2);
        if(beta_ < 0.0 || beta_ > 1.0) beta_ = 0.2; //Default in case you try to set a bad value
        
        pnh_.param("distance_threshold", d_f_, 0.5);
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
        pnh_.param("enable_radar_enhancement", enable_radar_enhancement_, false);
        pnh_.param("fuse_only_enhaced_radar", fuse_only_enhaced_radar_, false);
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
                  << "\tLidar Ransac Distance Threshold:   " << ransac_distance_threshold_lidar_ << std::endl
                  << "\tLidar Ransac Max Iters:            " << ransac_iterations_lidar_ << std::endl
                  << "\tLidar Ransac Min PointCloud Size:  " << min_ransac_pointcloud_size_lidar_ << std::endl
                  << "\tRadar Std Dev (m):                 " << radar_dev_ << std::endl
                  << "\tRadar Ransac Distance Threshold:   " << ransac_distance_threshold_radar_ << std::endl
                  << "\tRadar Ransac Max Iters:            " << ransac_iterations_radar_ << std::endl
                  << "\tAcc. Radar Enabled:                " << std::boolalpha << enable_radar_enhancement_ << std::endl
                  << "\tAcc. Radar Fuse only Acc Radar:    " << fuse_only_enhaced_radar_ << std::endl
                  << "\tAcc. Radar Max Yaw Diff:           " << max_radar_acc_yaw_incr_ << std::endl
                  << "\tInflate Radar Using Ang. Res:      " << inflate_radar_points_from_angular_resolution_ << std::endl
                  << "\tAngular Resolution Yaw (deg):      " << yaw_res_deg << std::endl
                  << "\tAngular Resolution Azimuth (deg):  " << azim_res_deg << std::endl
                  << "\tAngular inflation points step(m):  " << point_step_ << std::endl
                  << "\tBase Frame ID:                     " << robot_base_frame_id_ << std::endl
                  << "\tOdom Frame ID:                     " << odom_frame_id_ << std::endl
                  << "\033[1;31m---------------------------------------------\033[0m" << std::endl;
        if(save_radar_data_)
            radar_data_file_.open ("/home/fali/radar_data.txt");

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
    template<typename T=pcl::PointXYZI>
    pcl::PointCloud<T> processRadar(pcl::PointCloud<T> &points, const std::string _frame_id){
        
        typename pcl::PointCloud<T> int_cloud = points;
        std::vector<double> radar_ranges;
        double radar_max_range = std::numeric_limits<double>::max();

        if(inflate_radar_points_from_angular_resolution_)
            inflateRadarScanByResolution(int_cloud);
        
        for (auto &it : int_cloud)
            radar_ranges.push_back(std::move(dist2Origin(it)));

        double average_range_radar_scan_ = std::accumulate(radar_ranges.begin(), radar_ranges.end(), 0.0) / radar_ranges.size();
        std::cout << "Processed Radar" << std::endl;

        auto max = std::max_element(radar_ranges.begin(), radar_ranges.end());
        if( max != radar_ranges.end() )
            radar_max_range = *max;
        
        overlapping_scan_field_r_f_ = average_range_radar_scan_ + beta_ * (radar_max_range - average_range_radar_scan_);
        std::cout << "Processed Radar" << std::endl;
        
        std_msgs::Float32 overlapping_dist;
        overlapping_dist.data = overlapping_scan_field_r_f_;
        overlapping_dist_pub_.publish(overlapping_dist);
        std::cout << "Processed Radar" << std::endl;

        auto cloud_ransac = applyLineRANSAC(int_cloud, _frame_id, ransac_iterations_radar_, min_ransac_pointcloud_size_radar_, ransac_distance_threshold_radar_);
        
        if(ransac_iterations_radar_ > 0){
            cloud_ransac.header.frame_id = _frame_id;
            pcl_conversions::toPCL(ros::Time::now(), cloud_ransac.header.stamp);
            radar_ransac_result_pub_.publish(cloud_ransac);
        }
        std::cout << "Processed Radar" << std::endl;
        return cloud_ransac;
    }   
    
    void processLidar(const pcl::PointCloud<pcl::PointXYZ> &points, const std::string _frame_id)
    {
        virtual_2d_scan_cloud_ = extract2DVirtualScans(points, _frame_id);

        lidar_ransac_result_cloud_ = applyLineRANSAC(virtual_2d_scan_cloud_, _frame_id, ransac_iterations_lidar_, min_ransac_pointcloud_size_lidar_, ransac_distance_threshold_lidar_);
        
        if(ransac_iterations_lidar_ > 0){
            lidar_ransac_result_cloud_.header.frame_id = _frame_id;
            pcl_conversions::toPCL(ros::Time::now(), lidar_ransac_result_cloud_.header.stamp);
            lidar_ransac_result_pub_.publish(lidar_ransac_result_cloud_);
        }
    }
    void pointCloudsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg,
                             const sensor_msgs::PointCloud2::ConstPtr& radar_msg)
    {   
        sensor_msgs::PointCloud2  radar_msg_odom_frame, radar_msg_lidar_frame;
        pcl::PointCloud<pcl::PointXYZI> radar_cloud_odom, radar_cloud_lidar_frame;

        ros::Duration time_diff = lidar_msg->header.stamp - radar_msg->header.stamp;

        radar_original_frame_ = radar_msg->header.frame_id;
             
        pcl_ros::transformPointCloud(odom_frame_id_, *radar_msg, radar_msg_odom_frame,tf_listener_);
        pcl::fromROSMsg(radar_msg_odom_frame, radar_cloud_odom);
        acc_radar_clouds_ += radar_cloud_odom;
        
        if(fuse_only_enhaced_radar_ && !insert_acc_radar_clouds_)  return; //Not enough radar clouds yet, skip

        if(time_diff.toSec() > time_tolerance_){
            ROS_WARN("Skipping pair of lidar-radar clouds because they exceed time tolerance %f > %f", time_diff.toSec(), time_tolerance_);
            return;
        }

        pcl::fromROSMsg(*lidar_msg, last_lidar_cloud_);

        pcl_ros::transformPointCloud(lidar_msg->header.frame_id, *radar_msg, radar_msg_lidar_frame,tf_listener_);
        pcl::fromROSMsg(radar_msg_lidar_frame, last_radar_cloud_lidar_frame_);

        if(save_radar_data_){
            std::pair<double,double> temp_p;
            for(auto &it: last_radar_cloud_lidar_frame_){
                temp_p = xyToPolar(it);
                radar_data_file_ << temp_p.first << ", " << temp_p.second << ", "  << it.intensity << std::endl;
            }
        }
        
        processLidar(last_lidar_cloud_, lidar_msg->header.frame_id);
      
        //To apply Eq2 of the paper (fuseRanges function) wee need to iterate the lidar and the radar clouds 
        //We need to search for points that verifies that |R_lidar - R_radar| < d_F
        //In case two range measurements verify this inequality, calculate R fusion and put it into the result fused cloud
        if(enable_radar_enhancement_ && insert_acc_radar_clouds_ ){
            pcl::PointCloud<pcl::PointXYZI> radar_acc_ransac_result_cloud;
            auto interm_cloud = processRadar(acc_radar_clouds_base_frame_, acc_radar_clouds_base_frame_.header.frame_id);
            std::cout << "Interm cloud timestamp: " << interm_cloud.header.stamp << std::endl;
            interm_cloud.header.stamp -=  100000; //! This magic is done to avoid extrapolation into the future TF ERROR (dont know why it happens)
            
            pcl_ros::transformPointCloud(lidar_msg->header.frame_id, interm_cloud, radar_acc_ransac_result_cloud,tf_listener_);
            radar_acc_ransac_result_cloud.header.frame_id = lidar_msg->header.frame_id;
            pcl_conversions::toPCL(ros::Time::now(), radar_acc_ransac_result_cloud.header.stamp);
            
            fused_scan_result_ = createFusedCloud(lidar_msg->header.frame_id, lidar_ransac_result_cloud_, radar_acc_ransac_result_cloud);
            
            std::cout<< "Publishing fused cloud with radar enahcement!" << radar_acc_ransac_result_cloud.points.size() << " /" << fused_scan_result_.points.size() << std::endl;
            
            insert_acc_radar_clouds_ = false;
            acc_radar_clouds_.points.clear();
        }else{
            radar_ransac_result_cloud_ = processRadar(last_radar_cloud_lidar_frame_, lidar_msg->header.frame_id);
            fused_scan_result_ = createFusedCloud(lidar_msg->header.frame_id, lidar_ransac_result_cloud_, radar_ransac_result_cloud_);
        }

         std::cout << "Cloud sizes:" << std::endl <<
                     "  Lidar: " << lidar_ransac_result_cloud_.points.size() << std::endl <<
                     "  Radar: " << last_radar_cloud_lidar_frame_.points.size() << std::endl <<
                     "  Radar After Ransac: " << radar_ransac_result_cloud_.points.size() << std::endl;
    }
    template<typename T=pcl::PointXYZ, typename U=pcl::PointXYZI>
    pcl::PointCloud<pcl::PointXYZ> createFusedCloud(const std::string &frame_id, const pcl::PointCloud<T> &lidar_cloud, const pcl::PointCloud<U> &radar_cloud){


        //It's more efficient to iterate over radar cloud as it has less points that the lidar cloud (? sure ?)
        std::map<int, std::pair<double, double>> lidar_ranges_polar_coord_map;
        std::cout<< "Ransac result cloud size: " << lidar_ransac_result_cloud_.points.size() <<std::endl; 
        int i = 0;
        std::pair<int, std::pair<double,double>> temp_pair(0,std::pair<double,double>(0,0));
        // std::pair<int, std::pair<double,double>> temp_pair(0,std::pair<double,double>(lidar_max_range_ + 1,0));
        //for(int j = 0; j < std::floor(360/yaw_tolerance_); ++j){
        //    temp_pair.first = j;
        //    lidar_ranges_polar_coord_map.insert(temp_pair);
        //}
        temp_pair.first = 0;
        for(auto &it: lidar_cloud){//TODO RE THINK ON HOW TO BUILD THIS LIDAR RANGES POLAR COORD MAP (IS LIKE A SENSOR MSG LASER SCAN)
            ++temp_pair.first;      //I think it's better to build a full scan initialized with all the points at inf. 
            temp_pair.second = xyToPolar(it); //And after all replace the inf points but the existing values, but the result scan 
            lidar_ranges_polar_coord_map[temp_pair.first] =  temp_pair.second;
            // lidar_ranges_polar_coord_map.insert(temp_pair);
        }

        if( lidar_ranges_polar_coord_map.empty() ) ROS_WARN("Lidar Cloud empty!!!");

        std::vector<std::pair<double, double>> fused_ranges, lidar_ranges, radar_ranges;
        std::vector<int> fused_lidar_points_list; //Here we store the key numbers of the lidar_ranges_polar_coord_map that have been already inserted anywhere
                                                  //To avoid duplicated points
        double radar_point_range, fused_range;        
        
        //TODO There are repeated points pushed inside the final container. FIX
        for(auto &it: radar_cloud){
            radar_point_range = dist2Origin(it);
            if(lidar_ranges_polar_coord_map.empty()) break; //If no lidar cloud, exit and fill the result cloud with the radar poinnts
            
            for(auto &lidar_range: lidar_ranges_polar_coord_map){
                if(std::find(fused_lidar_points_list.begin(), fused_lidar_points_list.end(), lidar_range.first) != fused_lidar_points_list.end()) continue; //Skip lidar point if already considered
                //!Lidar insertion conditions
                if( std::fabs( lidar_range.second.first - radar_point_range ) > d_f_ ){
                    lidar_ranges.emplace_back(lidar_range.second);
                    fused_lidar_points_list.push_back(lidar_range.first);
                    continue;
                }

                fused_range = fuseRanges(radar_point_range, radarDev(), lidar_range.second.first, lidarDev(lidar_range.second.first));

                if( lidar_range.second.first > overlapping_scan_field_r_f_ && lidar_range.second.first < lidar_max_range_ ){ //120 == infinity
                    lidar_ranges.emplace_back(lidar_range.second);
                    fused_lidar_points_list.push_back(lidar_range.first);
                    continue;
                }
                if( lidar_range.second.first - radar_point_range > d_f_ && lidar_range.second.first < fused_range){
                    lidar_ranges.emplace_back(lidar_range.second);
                    fused_lidar_points_list.push_back(lidar_range.first);
                    continue;
                }
                //! Radar Insertion conditions
                    //? Type I
                if( lidar_range.second.first - radar_point_range < -1*d_f_ && radar_point_range < overlapping_scan_field_r_f_ ){ //Caso radar un poco mas alejado que lidar
                    radar_ranges.emplace_back(std::move(std::pair<double, double>(radar_point_range, pointYaw(it))));
                    continue;
                }
                    //? Type II
                if( lidar_range.second.first >= lidar_max_range_ && radar_point_range < lidar_max_range_ ){ //120 == infinity Caso lidar dando infinito(no se da?) 
                    radar_ranges.emplace_back(std::move(std::pair<double, double>(radar_point_range, pointYaw(it))));
                    continue;
                }
                //! Fused ranges insertion condition
                if( std::fabs(lidar_range.second.first - radar_point_range) < d_f_ ){
                    fused_ranges.emplace_back(std::move(std::pair<double,double>(fused_range, lidar_range.second.second)));
                    fused_lidar_points_list.push_back(lidar_range.first);
                    continue;
                }                
            }
        }

        //In case lidar cloud is empty, fill the result cloud with the radar points (VERY HIGH DENSITY SMOKE CASE)
        if(lidar_ranges_polar_coord_map.empty()){
            for(auto &it: radar_cloud){
                std::pair<double, double> p(dist2Origin(it), pointYaw(it));
                radar_ranges.emplace_back(p);
            }
        }
        if(radar_cloud.empty()){
            for(const auto &it: lidar_ranges_polar_coord_map)
                lidar_ranges.push_back(it.second);
        }
        //Undo ranges calculation-> get x,y points coordinates from polar ones

        //Create cloud from fused ranges
        pcl::PointCloud<pcl::PointXYZ> result_cloud, fused_points_cloud, radar_points_cloud, lidar_points_cloud;
        pcl::PointXYZ p;
        
        result_cloud.header.frame_id = frame_id;     
        fused_points_cloud.header.frame_id = frame_id;
        radar_points_cloud.header.frame_id = frame_id;
        lidar_points_cloud.header.frame_id = frame_id;

        p.z = 0;
        //Undo fused ranges cloud
        for(const auto &it: fused_ranges)
            fused_points_cloud.push_back(std::move(fromPolarToXY(it)));
        
        //Undo radar ranges cloud
        for(const auto &it: radar_ranges)
            radar_points_cloud.push_back(std::move(fromPolarToXY(it)));
        
        //Undo lidar ranges cloud
        for(const auto &it: lidar_ranges)
            lidar_points_cloud.push_back(std::move(fromPolarToXY(it)));
        

        pcl_conversions::toPCL(ros::Time::now(), lidar_points_cloud.header.stamp);
        pcl_conversions::toPCL(ros::Time::now(), radar_points_cloud.header.stamp);
        pcl_conversions::toPCL(ros::Time::now(), fused_points_cloud.header.stamp);

        fused_points_cloud_pub_.publish(fused_points_cloud);
        radar_points_cloud_pub_.publish(radar_points_cloud);        
        lidar_points_cloud_pub_.publish(lidar_points_cloud);
        
        std::cout <<"Radar contributions: " << radar_points_cloud.points.size()
                  <<" Lidar: " << lidar_points_cloud.points.size()  
                  <<" Fused: " << fused_points_cloud.points.size() << std::endl;

        //Put all three clouds together      
        result_cloud += fused_points_cloud;
        result_cloud += lidar_points_cloud;
        result_cloud += radar_points_cloud; 
        pcl_conversions::toPCL(ros::Time::now(), result_cloud.header.stamp);
        final_points_cloud_pub_.publish(result_cloud);

        return result_cloud;
    }
    //According to the paper the sigma_R is proportional to 1/Pe with Pe received power. Pe should be proportional to 1/R2 ?
    template <typename T>
    std::pair<double, double> xyToPolar(const T &point){
        return std::make_pair<double, double>(dist2Origin(point), pointYaw(point));
    }
    template <typename T>
    void fromPolarToXY(const std::pair<double, double>  &point , T &xy_point){
        xy_point.x = point.first*cos(point.second);
        xy_point.y = point.first*sin(point.second);
    }
    template <typename T=pcl::PointXYZ>
    T fromPolarToXY(const std::pair<double, double> &point){
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
    template <typename T>
    double dist2Origin(const T &point)
    {
        return sqrtf(point.x * point.x + point.y * point.y + point.z * point.z);
    }
    template <typename T>
    double pointYaw(const T &point)
    {
        if (point.y == 0.0 && point.x == 0)
            return 0;

        return atan2(point.y, point.x);
    }
    template <typename T>
    void pointToSpherical(const T &_point, double &_rho, double &_theta, double &_phi){
        _theta = atan2(_point.y, _point.x);
        _rho   = dist2Origin(_point);
        _phi   = acos(_point.z/_rho);
    }
    template <typename T=pcl::PointXYZI>
    T sphericalToXY(const double &_rho, const double &_theta, const double &_phi, bool z_zero = true){
        T p;
        p.x = _rho * sin(_phi) * cos(_theta);
        p.y = _rho * sin(_phi) * sin(_theta);
        if(!z_zero)
            p.z = _rho * cos(_theta);

        return p;
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
        std::vector<pcl::PointXYZ> last_points;
        pcl::PointXYZ point;

        out.header.frame_id = frame_id;
        double yaw          = 0;

        //A virtual 2D scan that contains primarily walls  can  thereafter  be  assembled  by  taking  one  point  out
        //of  each vertical raw scan (resp. two points for a yawing scan top). This point is chosen to be the one with
        //the largest distant to the center  of  the  robot.
        for (auto &it : cloud)
        {
            point.x = it.x;
            point.y = it.y;

            if (last_points.empty())
            {
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
                    if (dist2Origin(it) > dist2Origin(final_point))
                        final_point = it;
                
                last_points.clear();
                out.push_back(final_point);
            }
        }
        pcl_conversions::toPCL(ros::Time::now(), out.header.stamp);
        virtual_2d_pointcloud_pub_.publish(out);

        return out;
    }
    template <typename T>
    pcl::PointCloud<T> applyLineRANSAC(pcl::PointCloud<T> &cloud, const std::string &frame_id,
                                                   const int _ransac_iterations, const int _min_pointcloud_size, const double _ransac_distance_threshold)
    {

        if(_ransac_iterations == 0)
            return cloud;

        typename pcl::PointCloud<T>::Ptr out_cloud(new pcl::PointCloud<T>);
        typename pcl::PointCloud<T>::Ptr in_intermediate(new pcl::PointCloud<T>);
        
        typename pcl::SampleConsensusModelLine<T>::Ptr model_l;
        typename pcl::RandomSampleConsensus<T>::Ptr ransac;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        typename pcl::ExtractIndices<T> extract;

        *in_intermediate = cloud;

        for (int i = 0; i < _ransac_iterations; ++i)
        {
            if(in_intermediate->points.size() < _min_pointcloud_size ) break; //It should be nice here to check the standard deviation 
                                                                                     // of remaining points to see how "spreaded" they are
                                                                                     // and continue iterating if they not much spreaded?

            model_l.reset(new pcl::SampleConsensusModelLine<T>(in_intermediate));
            ransac.reset(new pcl::RandomSampleConsensus<T>(model_l));
            inliers.reset(new pcl::PointIndices());

            ransac->setDistanceThreshold(_ransac_distance_threshold);
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

        return *out_cloud;
    }
    //! This functions takes points from a PointCloud and inflated in SPHERICAL coordinates along
    //! the yaw (theta) and elevation(phi) angles resulting in a "spherical shell inflation"
    //! Other possibility is to put all them into a plane
    template<typename T=pcl::PointXYZI>
    void inflateRadarScanByResolution(pcl::PointCloud<T> &in_cloud){
        //TODO How to check if there is intensity field and forward it?
        
        typename pcl::PointCloud<T> int_cloud;
        T point;
        int k1,k2,k3; //Yaw(theta)//Azimuth (phi)//Range(m)
        double theta_n, phi_n, rho_n,theta, phi, rho;

        int_cloud.header = in_cloud.header;

        for(auto &it: in_cloud){
            pointToSpherical(it, rho,theta,phi);
            k1    = std::fabs(std::round(2*rho*sin(radar_angular_resolution_yaw_) / point_step_));
            k2    = std::fabs(std::round(2*rho*sin(radar_angular_resolution_azimuth_) / point_step_));
            k3    = std::fabs(std::round(radar_dev_ / point_step_));
            if(k1 % 2 != 0) k1 -= 1; 
            if(k2 % 2 != 0) k2 -= 1; 
            if(k1 < 2) k1 = 2;
            if(k2 < 2) k2 = 2;
            for(int k = -k3; k <= k3; k++){
                rho_n = rho + k * point_step_;
                for(int i = 0; i < k1; i++ ){
                    theta_n = theta - radar_angular_resolution_yaw_/2 + i * radar_angular_resolution_yaw_/k1;
                    for(int j = 0; j < k2; j++ ){
                        phi_n   = phi  - radar_angular_resolution_azimuth_/2 + j* radar_angular_resolution_azimuth_/k2;
                        int_cloud.points.push_back(std::move(sphericalToXY<T>(rho_n,theta_n,phi_n)));   
                    }
                }
            }
        }

        pcl_conversions::toPCL(ros::Time::now(), int_cloud.header.stamp);
        inflated_radar_cloud_pub_.publish(int_cloud);
        in_cloud = int_cloud;
    }
    void dynamicReconfigureCallback(radar_experiments::LidarFusionConfig &config, uint32_t level){
        if(first){
            first = false;
            return;
        }
        yaw_tolerance_                    = config.yaw_tolerance;
        beta_                             = config.beta;
        d_f_                              = config.distance_threshold;
        time_tolerance_                   = config.time_tolerance;
        radar_dev_                        = config.radar_dev;
        lidar_dev_                        = config.lidar_dev;
        ransac_iterations_lidar_          = config.lidar_ransac_iterations;
        min_ransac_pointcloud_size_lidar_ = config.lidar_min_ransac_pointcloud_size;
        ransac_distance_threshold_lidar_  = config.lidar_ransac_distance_threshold;
        ransac_iterations_radar_          = config.radar_ransac_iterations;
        min_ransac_pointcloud_size_radar_ = config.radar_min_ransac_pointcloud_size;
        ransac_distance_threshold_radar_  = config.radar_ransac_distance_threshold;

        ROS_INFO("Dynamic reconfigure called!");
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg){

        if(!enable_radar_enhancement_){
            odom_sub_.shutdown();
            return;
        }

        double d_yaw = std::fabs(atan2(msg->pose.pose.position.y,msg->pose.pose.position.x) - first_yaw_); 

        if(ros::Time::now() - first_odom_time_ > ros::Duration(max_radar_acc_time_) || 
           d_yaw > max_radar_acc_yaw_incr_ ){
            new_odom_vec_ = false;
            first_yaw_    = atan2(msg->pose.pose.position.y,msg->pose.pose.position.x);
            std::cout << "First yaw measure: " << first_yaw_ << std::endl;
        }
        if(!new_odom_vec_){
            //Publish radar cloud acc before clearing:
            acc_radar_clouds_.header.frame_id = odom_frame_id_;
            
            pcl_ros::transformPointCloud(robot_base_frame_id_, acc_radar_clouds_, acc_radar_clouds_base_frame_,tf_listener_);
            acc_radar_clouds_base_frame_.header.frame_id = robot_base_frame_id_;
            
            pcl_conversions::toPCL(ros::Time::now(), acc_radar_clouds_base_frame_.header.stamp);
            accumulated_radar_cloud_pub_.publish(acc_radar_clouds_base_frame_);

            odom_msgs_.clear();
            
            first_odom_time_         = ros::Time::now();
            new_odom_vec_            = true;
            insert_acc_radar_clouds_ = true;
        }
        
        odom_msgs_.push_back(*msg);
        std::cout << "Odom msgs size: " << odom_msgs_.size() << std::endl;
        
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
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> LidarRadarSyncPolicy; 
    
    message_filters::Synchronizer<LidarRadarSyncPolicy> *pointclouds_subs_sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub_lidar_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub_radar_;
    
    ros::NodeHandle pnh_{"~"};
    ros::AsyncSpinner spinner;
    ros::Subscriber odom_sub_;
    ros::Publisher  virtual_2d_pointcloud_pub_;
    ros::Publisher  lidar_ransac_result_pub_, radar_ransac_result_pub_;
    ros::Publisher  overlapping_dist_pub_;
    ros::Publisher  fused_points_cloud_pub_;
    ros::Publisher  radar_points_cloud_pub_;
    ros::Publisher  lidar_points_cloud_pub_;
    ros::Publisher  final_points_cloud_pub_;
    ros::Publisher  accumulated_radar_cloud_pub_;
    ros::Publisher  inflated_radar_cloud_pub_;
    ros::Publisher  ransac_lines_pub_;
    tf::TransformListener tf_listener_;

    std::unique_ptr<dynamic_reconfigure::Server<radar_experiments::LidarFusionConfig>> dynamic_reconf_server_;
    std::unique_ptr<dynamic_reconfigure::Server<radar_experiments::LidarFusionConfig>::CallbackType> dynamic_recong_cb_f_;
    bool first{true}; //Used to detect the init of the server, when it is initialized it calls the callback at the startup and overrides the launch params(whe dont want this)

    pcl::PointCloud<pcl::PointXYZI> last_radar_cloud_lidar_frame_, radar_ransac_result_cloud_;
    pcl::PointCloud<pcl::PointXYZ> last_lidar_cloud_;
    pcl::PointCloud<pcl::PointXYZ> virtual_2d_scan_cloud_;
    pcl::PointCloud<pcl::PointXYZ> lidar_ransac_result_cloud_;
    //! Cloud to store final result
    pcl::PointCloud<pcl::PointXYZ> fused_scan_result_;
    //! Radar enhancement
    std::vector<nav_msgs::Odometry> odom_msgs_;
    ros::Time first_odom_time_;
    bool new_odom_vec_{true};
    bool enable_radar_enhancement_;
    bool fuse_only_enhaced_radar_;
    bool insert_acc_radar_clouds_{false};
    double first_yaw_{0};
    pcl::PointCloud<pcl::PointXYZI> acc_radar_clouds_;
    std::string odom_frame_id_, robot_base_frame_id_;
    pcl::PointCloud<pcl::PointXYZI> acc_radar_clouds_base_frame_;
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
    ros::init(argc, argv, "lidar_radar_sensor_fuser");

    LidarRadarFuser fuser;

    fuser.run();

    return 0;
}