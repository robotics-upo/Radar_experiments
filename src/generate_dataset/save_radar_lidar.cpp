//! Basic fusion based on gaussian mixture of Radar measures

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>

class SaveRadarLidar
{
    using DoublePair = std::pair<double, double>;
public:
    SaveRadarLidar(): spinner(2), pointclouds_subs_sync_(NULL)
    {
        //Lidar and Radar pointcloud sync subscriber
        pnh_.param("sychronization_margin", synch_margin_queue_,10);
        pnh_.param("filename", filename_, std::string("dataset_"));
        pnh_.param("max_range", max_range_, 20.0);
        pnh_.param("separator", separator_, std::string(" "));

        pnh_.param("min_radar_measures", min_radar_measures_, 5);

        pc_sub_lidar_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(pnh_, "/lidar_scan", 1));
        pc_sub_radar_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(pnh_, "/radar_scan", 1));
        pc_sub_ground_truth_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(pnh_, "/ground_truth_scan", 1));

        pointclouds_subs_sync_ = new message_filters::Synchronizer<LidarRadarSyncPolicy>(LidarRadarSyncPolicy(synch_margin_queue_), *pc_sub_lidar_, *pc_sub_radar_, *pc_sub_ground_truth_);
        pointclouds_subs_sync_->registerCallback(&SaveRadarLidar::laserCallback, this);

        pnh_.param("time_tolerance", time_tolerance_, 1.0);
        if(time_tolerance_ < 0) time_tolerance_ *= -1;
    }
    
    
    void run()
    {
        spinner.start();
        ros::waitForShutdown();
    }
private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& lidar_msg,
                             const sensor_msgs::LaserScan::ConstPtr& radar_msg,
                             const sensor_msgs::LaserScan::ConstPtr& gt_msg)
    {               // We will take as inputs laser scans obtained from the PC
        auto time_diff = lidar_msg->header.stamp - radar_msg->header.stamp;
        auto time_diff2 = gt_msg->header.stamp - radar_msg->header.stamp;
        auto time_diff3 = gt_msg->header.stamp - lidar_msg->header.stamp;
        if(fabs(time_diff.toSec()) > time_tolerance_ ||
           fabs(time_diff2.toSec()) > time_tolerance_ || 
           fabs(time_diff3.toSec()) > time_tolerance_ )   {
            ROS_WARN("Skipping pair of lidar-radar clouds because they exceed time tolerance %f > %f", time_diff.toSec(), time_tolerance_);
            return;
        }

        ROS_INFO_ONCE("Got 3 SCANS --> saving file");

        // Write the messages to a file
        static unsigned int id = 0;
        std::ostringstream out;
        out << filename_ << std::setfill('0') << std::setw(8) << id;

        try {
            std::ofstream ofs(out.str());
        // Open a file and fill it with 1st the lidar scan, then the radar scan and finally the ground truth scan
            for (auto x: lidar_msg->ranges) {
                if (x < max_range_) {
                    ofs << x << separator_;
                } else {
                    ofs << 0.0 << separator_;
                }
            }
            ofs << "\n\n";
            int  cont = 0;
            for (auto x: radar_msg->ranges) {
                if (x < max_range_) {
                    cont++;
                    ofs << x << separator_;
                } else {
                    ofs << 0.0 << separator_;
                }
            }
            ROS_INFO("Radar measures: %d", cont);

            if (cont < min_radar_measures_) {
                return;
            }
            ofs << "\n\n";
            for (auto x: gt_msg->ranges) {
                if (x < max_range_) {
                    ofs << x << separator_;
                } else {
                    ofs << 0.0 << separator_;
                }
            }
        } catch (std::exception &e) {
            ROS_ERROR("Got an exception while writing to file. Content: %s", e.what());
        }
        id++;
    }
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> LidarRadarSyncPolicy; 

    //! Node Params
    int synch_margin_queue_;
    double time_tolerance_;
    double max_range_;
    int min_radar_measures_;
    std::string filename_, separator_;

    // ROS stuff
    message_filters::Synchronizer<LidarRadarSyncPolicy> *pointclouds_subs_sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> pc_sub_lidar_, pc_sub_radar_, pc_sub_ground_truth_;
    
    ros::NodeHandle pnh_{"~"};
    ros::AsyncSpinner spinner;
    ros::Publisher odom_sub_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_radar_lidar");

    class SaveRadarLidar fuser;

    fuser.run();

    return 0;
}