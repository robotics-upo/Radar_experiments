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

class LidarRadarFuser
{

public:
    LidarRadarFuser()
    {
        pointcloud_sub_             = pnh_.subscribe("/points", 1, &LidarRadarFuser::pointCloudCallback, this);

        virtual_2d_scan_pub_        = pnh_.advertise<sensor_msgs::LaserScan>("virtual_2d_scan", 1);
        virtual_2d_pointcloud_pub_  = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("virtual_2d_pointcloud", 1);
        ransac_result_pub_          = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("ransac_result", 1);

        //Params....
        pnh_.param("yaw_tolerance", yaw_tolerance_, 0.1);
        pnh_.param("ransac_distance_threshold", ransac_distance_threshold_, 0.01);
        pnh_.param("ransac_iterations", ransac_iterations_, 5);
        pnh_.param("min_ransac_pointcloud_size", min_ransac_pointcloud_size_, 10);
    
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

                yaw = atan2(it.y, it.x); //between [-pi,pi]
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
    sensor_msgs::LaserScan cloud2LaserScan(const pcl::PointCloud<pcl::PointXYZ> &points)
    {
    }

private:
    
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {

        pcl::fromROSMsg(*msg, last_cloud_);

        virtual_2d_scan_cloud_ = extract2DVirtualScans(last_cloud_, msg->header.frame_id);

        ransac_result_cloud_ = applyLineRANSAC(virtual_2d_scan_cloud_, msg->header.frame_id);
        // virtual_2d_laserscan_ = cloud2LaserScan(virtual_2d_scan_cloud_);
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


    ros::NodeHandle pnh_{"~"};
    ros::Subscriber pointcloud_sub_;
    ros::Publisher  virtual_2d_scan_pub_;
    ros::Publisher  virtual_2d_pointcloud_pub_;
    ros::Publisher  ransac_result_pub_;

    pcl::PointCloud<pcl::PointXYZ> last_cloud_;
    pcl::PointCloud<pcl::PointXYZ> virtual_2d_scan_cloud_;
    pcl::PointCloud<pcl::PointXYZ> ransac_result_cloud_;
    sensor_msgs::LaserScan virtual_2d_laserscan_;

    //!Params

    double yaw_tolerance_; // Used to extract virtual 2d scans. If two points have a yaw
                           // difference less than yaw_tolerance, we assume that they lie on the same
                           // vertical linea
    double ransac_distance_threshold_;
    int ransac_iterations_;
    int min_ransac_pointcloud_size_;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_radar_sensor_fuser");

    LidarRadarFuser fuser;

    ros::spin();

    return 0;
}