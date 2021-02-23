#include <ros/ros.h>
#include <numeric>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

namespace UPO
{

    class LidarStdDevEstimation
    {

    public:
        LidarStdDevEstimation()
        {
            pointcloud_sub_ = lnh_.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/os1_cloud_node/points_non_dense", 1, &LidarStdDevEstimation::pointCloudCallback, this);
            
            //Parameters
            lnh_.param("start_range", start_range, 0.5);
            start_range*=start_range;
            lnh_.param("band_distance", band_distance, 0.5);
    
            lnh_.param("max_elements", max_elements, 500);
        }

    private:
        void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
        {
            std::cout << "Received point cloud... " << std::endl;

            for (int nIndex = 0; nIndex < cloud->points.size (); nIndex++ )
            {
                // find the squared distance from the origin.
                double point_dist_sq = (cloud->points[nIndex].x * cloud->points[nIndex].x) +
                                     (cloud->points[nIndex].y * cloud->points[nIndex].y) + 
                                     (cloud->points[nIndex].z * cloud->points[nIndex].z) ;
                
                //Calculate corresponding band(espheric shell)

            }
        }

        double computeStdDev(const std::vector<double> &vec)
        {
            //Dostuff
            std::cout << "Computing std dev... " << std::endl;

            double mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
            double sq_sum = 0;

            for (auto &it : vec)
                sq_sum += pow(it - mean, 2);

            std::cout << "Std dev: " << std::sqrt(sq_sum / vec.size()) << "\n\tUsed Size: " << vec.size() << std::endl;

            return std::sqrt(sq_sum / vec.size());
        }
        ros::NodeHandle lnh_{"~"};
        ros::Subscriber pointcloud_sub_;

        std::map<std::pair<double, double>, std::vector<double>> ranges_lists;
        //Parameters
        double start_range, band_distance;
        int max_elements;
    };
} // namespace UPO

int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar std dev estimation");

    UPO::LidarStdDevEstimation est;

    ros::spin();
    return 0;
}
