/*
 * Copyright (c) 2021, Service Robotics Lab
 * All rights reserved.
 *
 * See LICENSE file for distribution details
 */

#include <boost/algorithm/string.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "g2o/types/slam2d/types_slam2d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

// Basic make_unique helper function, similar to C++14 one
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/buffer_core.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <ros/ros.h>

using namespace std;
using namespace g2o;

void changeTf(tf2::Transform &t, double x, double y, double theta, bool rp_to_zero = false)  {
    t.setOrigin(tf2::Vector3(x, y, 0.0));
    auto q = t.getRotation();
    tf2::Matrix3x3 M(q);
    double roll, pitch, yaw;
    
    if (rp_to_zero) {
        roll = pitch = 0.0;
    } else {
        M.getRPY(roll, pitch, yaw);
    }
    q.setRPY(roll, pitch, theta);
    t.setRotation(q);
}

//! @class GroundTruthRefinar:

//! It gets the measures from the fiducial estimation and combines them with odometry measures obtaining a more 
//! filtered ground truth estimation
class GroundTruthRefiner {

    public:
    GroundTruthRefiner();


    void groundTruthCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose);

    bool updateOdometry(); // Whenever the base displaces over a threshold --> put an extra odometry measure

    void optimize(int n_rounds);

    bool exportPath(const std::string &filename) const;

    protected:
    // ROS Stuff
    ros::Subscriber _gt_sub;
    ros::Publisher _original_path_pub, _optimized_path_pub;
    tf2::Transform _last_tf_odom_base;
    geometry_msgs::PoseWithCovarianceStamped last_pose;
    tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> listener;

    // Paths:

    nav_msgs::Path original_path, optimized_path;

    // G2O STUFF

    // allocating the optimizer
    g2o::SparseOptimizer _optimizer;
    std::unique_ptr<SlamLinearSolver> _linear_solver;
    Eigen::Matrix3d _odometry_information, _observation_information;
    std::string _baseFrame, _odomFrame, _mapFrame;
    double _ang_thres, _dist_thres;

    std::string _g2o_file;

    int _frame_num = 0;

    // Ignoring bad estimation of ground truth
    std::string ignore_ids;
    std::set<int> ignore_ids_set;

    static void get_tf_coords(double &x, double &y, double &theta, const tf2::Transform &t);
    virtual bool lookupTransform(const std::string &from, const std::string &to, const ros::Time &time,
                         tf2::Transform &T) const;
};

void GroundTruthRefiner::get_tf_coords(double &x, double &y, double &theta, const tf2::Transform &t) {
    x = t.getOrigin().getX();
    y = t.getOrigin().getY();
    auto q = t.getRotation();
    tf2::Matrix3x3 M(q);
    double roll,pitch;
    M.getRPY(roll, pitch, theta);
}

// lookup specified transform
bool GroundTruthRefiner::lookupTransform(const std::string &from, const std::string &to, const ros::Time &time,
                          tf2::Transform &T) const {
    geometry_msgs::TransformStamped transform;

    try {
        // transform = tfBuffer.lookupTransform(from, to, time);
        transform = tfBuffer.lookupTransform(from, to, ros::Time(0));

        tf2::fromMsg(transform.transform, T);
        tf2::Vector3 c = T.getOrigin();


        return !isnan(c.x()) && !isnan(c.y()) && !isnan(c.z());
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }
}

GroundTruthRefiner::GroundTruthRefiner():tfBuffer(ros::Duration(30.0)) {
    // If set, use the fiducial area in pixels^2 as an indication of the
    // 'goodness' of it. This will favor fiducials that are close to the
    // camera and center of the image. The reciprical of the area is actually
    // used, in place of reprojection error as the estimate's variance
    ros::NodeHandle nh, pnh("~");

    listener = std::make_unique<tf2_ros::TransformListener>(tfBuffer);

    _gt_sub = nh.subscribe("/gt_pose", 1, &GroundTruthRefiner::groundTruthCallback, this);
    _original_path_pub = ros::Publisher(pnh.advertise<nav_msgs::Path>("/original_path", 10));
    _optimized_path_pub = ros::Publisher(pnh.advertise<nav_msgs::Path>("/optimized_path", 10));

    // G2O initialization
    _linear_solver = std::make_unique<SlamLinearSolver>();
    _linear_solver->setBlockOrdering(false);
    g2o::OptimizationAlgorithmGaussNewton * solver =new g2o::OptimizationAlgorithmGaussNewton(::make_unique<SlamBlockSolver>(std::move(_linear_solver)));
    _optimizer.setAlgorithm(solver);

    if (pnh.hasParam("stats_file")) {
        pnh.getParam("stats_file", _g2o_file);
    } else {
        _g2o_file = "";
    }

    if (pnh.hasParam("ignore_ids")) {
        pnh.getParam("ignore_ids", ignore_ids);
        
        std::vector<std::string> results;

        boost::split(results, ignore_ids, boost::is_any_of(","));

        for (auto x:results) {

            if (x.find('-') != std::string::npos ) {
                std::vector<std::string> results;

                boost::split(results, x, boost::is_any_of("-"));

                if (results.size() == 2) {
                    cout << "Ignoring ids from: " << results[0] << " to " << results[1] << endl;
                    for (int cont = stoi(results[0]); cont <= std::stoi(results[1]); cont ++) {
                        ignore_ids_set.insert(cont);
                    } 
                }
            } else {

                ignore_ids_set.insert(std::stoi(x));
                cout << "Ignoring id: " << x << endl;
            }
        }

    }


    // Frames
    pnh.param<std::string>("map_frame", _mapFrame, "map");
    pnh.param<std::string>("odom_frame", _odomFrame, "odom");
    pnh.param<std::string>("base_frame", _baseFrame, "base_link");

    // Information of odometry, etc.
    double rot_noise, trans_noise_x, trans_noise_y;
    pnh.param<double>("rot_noise", rot_noise, 0.05);
    pnh.param<double>("trans_noise_x", trans_noise_x, 0.05);
    pnh.param<double>("trans_noise_y", trans_noise_y, 0.02);
    Eigen::Matrix3d covariance;
    covariance.fill(0.);
    covariance(0, 0) = trans_noise_x*trans_noise_x;
    covariance(1, 1) = trans_noise_y*trans_noise_y;
    covariance(2, 2) = rot_noise*rot_noise;
    _odometry_information = covariance.inverse();

    // Observation information:
    pnh.param<double>("rot_noise_obs", rot_noise, 0.1);
    pnh.param<double>("trans_noise_obs", trans_noise_x, 0.2);
    covariance(0, 0) = trans_noise_x*trans_noise_x;
    covariance(1, 1) = trans_noise_x*trans_noise_x; // NOTE: not a bug, same noise for x and y
    covariance(2, 2) = rot_noise*rot_noise;
    _observation_information = covariance.inverse();

    // Thresholds for updating odometry
    pnh.param<double>("dist_thres", _dist_thres, 0.1);
    pnh.param<double>("ang_thres", _ang_thres, 0.1);

    ROS_INFO("G2O Refine Ground Truth ready");

    // Add the first vertex at the origin (0,0,0) in order to be able to include estimated pose edges
    const SE2 origin_pose(0.0, 0.0, 0.0);
    VertexSE2* origin = new VertexSE2;
    origin->setId(0);
    origin->setEstimate(origin_pose);
    _optimizer.addVertex(origin);

    _frame_num = 1;
}

void GroundTruthRefiner::groundTruthCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
    
    if (ignore_ids_set.count(pose->header.seq)) {
        ROS_INFO("Ignoring groundtruth with id: %d", pose->header.seq);
        return;
    }

    
    
    tf2::Quaternion q;
    tf2::fromMsg(pose->pose.pose.orientation, q);
    tf2::Matrix3x3 M(q);
    double roll,pitch, theta;
    M.getRPY(roll, pitch, theta);

    double x = pose->pose.pose.position.x;
    double y = pose->pose.pose.position.y;
    ROS_INFO("Received pose. Coords: (%f, %f, %f)", x, y, theta);

    // Check for unusual divergence
    if (_frame_num > 1) {
        auto t = pose->header.stamp - last_pose.header.stamp;

        if (t.toNSec() < 100000000) {
            
            double dx = x - last_pose.pose.pose.position.x;
            double dy = y - last_pose.pose.pose.position.y;
            double dist = sqrt(dx*dx + dy*dy);
            if ( dist > 0.3 ) {
                ROS_INFO("Discarding pose for divergence");
                return;
            }
        }


        
    }
    


    const SE2 robot_pose(x, y, theta);
    VertexSE2* robot = new VertexSE2;
    robot->setId(_frame_num);
    robot->setEstimate(robot_pose);
    _optimizer.addVertex(robot);

    // Add observation constraint (between origin and the vertex)
    EdgeSE2* observation = new EdgeSE2;
    observation->vertices()[0] = _optimizer.vertex(0);
    observation->vertices()[1] = _optimizer.vertex(_frame_num);
    
    const SE2 obs_meas(x, y, theta);
    observation->setMeasurement(obs_meas);
    observation->setInformation(_observation_information);  // TODO: how to estimate?
    _optimizer.addEdge(observation);

    // Get the current pose of the vehicle
    tf2::Transform tf_odom_base;
    if (!lookupTransform(_odomFrame, _baseFrame, ros::Time(0), tf_odom_base))
        return;

    if (_frame_num > 1) {
        // Not first frame --> add odometry constraint (no external map --> use odom constraint)
        EdgeSE2* odometry = new EdgeSE2;
        odometry->vertices()[0] = _optimizer.vertex(_frame_num - 1);
        odometry->vertices()[1] = _optimizer.vertex(_frame_num);

        auto odom_tf = _last_tf_odom_base.inverse() * tf_odom_base;
        get_tf_coords(x, y, theta, odom_tf);
        const SE2 odom_meas(x, y, theta);
        odometry->setMeasurement(odom_meas);
        odometry->setInformation(_odometry_information);
        _optimizer.addEdge(odometry);
    }

    // Update vars
    _frame_num++;
    _last_tf_odom_base = tf_odom_base;

    // Append new pose
    geometry_msgs::PoseStamped p;

    p.header = pose->header;
    p.header.stamp = ros::Time::now();

    // ROS_INFO("Got ground truth. Stamp: %ld. Stamp of the message: %ld", p.header.stamp.toNSec(), pose->header.stamp.toNSec());
    p.pose = pose->pose.pose;

    original_path.poses.push_back(p);
    last_pose = *pose;
}

bool GroundTruthRefiner::updateOdometry() {
    if (_frame_num <= 1)
        return false; // The first vertex has to be a measure

    tf2::Transform t_odom_base; // Before optimization --> get odom base
    if (!lookupTransform(_odomFrame, _baseFrame, ros::Time(0), t_odom_base)) {
        return false;
    }

    // Get the odometry step:
    auto t_odom_step = _last_tf_odom_base.inverse() * t_odom_base;
    auto d_ = t_odom_step.getOrigin().length();

    double x, y, theta;
    double inc_x, inc_y, inc_theta;
    get_tf_coords(inc_x, inc_y, inc_theta, t_odom_step);

    if (d_ > _dist_thres || fabs(inc_theta) > _ang_thres) {

        auto pos = dynamic_cast<VertexSE2 *>(_optimizer.vertex(_frame_num - 1))->estimate();

        tf2::Transform t;
        t.setOrigin(tf2::Vector3(pos[0],pos[1],0.0));
        tf2::Quaternion q;
        q.setRPY(0,0, pos[2]);
        t.setRotation(q);

        t = t * t_odom_step;
        get_tf_coords(x,y, theta, t);
        ROS_INFO("Add odometry. Inc: (%f,%f,%f). Prev.: (%f, %f, %f). Coords: (%f, %f, %f)", inc_x, inc_y, inc_theta, pos[0], pos[1], pos[2], x, y, theta);

        const SE2 robot_pose(x, y, theta);
        VertexSE2* robot = new VertexSE2;
        robot->setId(_frame_num);
        robot->setEstimate(robot_pose);
        _optimizer.addVertex(robot);

        // Register pose in original path
        geometry_msgs::PoseStamped p;
        p.header.stamp = ros::Time::now();
        p.header.seq = _frame_num;
        p.header.frame_id = _mapFrame;
        p.pose.position.x = x; p.pose.position.y = y;
        tf::Quaternion t_q;
        t_q.setRPY(0,0, theta);
        tf::quaternionTFToMsg(t_q,p.pose.orientation);
        
        original_path.poses.push_back(p);
        

        // Add odometry constraint
        // Not first frame --> add odometry constraint (no external map --> use odom constraint)
        EdgeSE2* odometry = new EdgeSE2;
        odometry->vertices()[0] = _optimizer.vertex(_frame_num - 1);
        odometry->vertices()[1] = _optimizer.vertex(_frame_num);

        const SE2 odom_meas(inc_x, inc_y, inc_theta);
        odometry->setMeasurement(odom_meas);
        odometry->setInformation(_odometry_information);
        _optimizer.addEdge(odometry);

        // Update
        _frame_num++;
        _last_tf_odom_base = t_odom_base;

    


        return true;
    }
    return false;
}

/*********************************************************************************
   * optimization
   ********************************************************************************/
void GroundTruthRefiner::optimize(int n_rounds) {
  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom

  if (_frame_num < 2 ) {
      ROS_INFO("Not enough Frames. Cancelling optimization. ");
      return;
  }

  VertexSE2* origin = dynamic_cast<VertexSE2*>(_optimizer.vertex(0));
  if (origin == nullptr) {
      ROS_ERROR("Could not get the first Robot Pose. Cancelling\n");
      return;
  }
  origin->setFixed(true);
  _optimizer.setVerbose(true);

  tf2::Transform t_end, t_odom_base; // Before optimization --> get odom base
  bool got_transform = lookupTransform(_odomFrame, _baseFrame, ros::Time(0), t_odom_base);

  ROS_INFO("Optimizing");
  _optimizer.initializeOptimization();
  _optimizer.optimize(n_rounds);
  ROS_INFO("Done Optimizing.");

  // Initialize path for visualization
  optimized_path.poses.clear();
  static int path_seq = 0;
  optimized_path.header.frame_id = _mapFrame;
  optimized_path.header.stamp = ros::Time::now();
  optimized_path.header.seq = path_seq++;

  original_path.header = optimized_path.header;


  for (int cont = 1; cont < _optimizer.vertices().size(); cont++) {

        auto y = dynamic_cast<VertexSE2 *> (_optimizer.vertices().at(cont));
        auto &pos = y->estimate();
        if (y == nullptr || y->id() == 0) continue;

        geometry_msgs::PoseStamped curr_pose;
        tf2::Transform t;
        changeTf(t, pos[0], pos[1], pos[2], true);
        curr_pose.pose.orientation = tf2::toMsg(t.getRotation());
        curr_pose.pose.position.x = pos[0];
        curr_pose.pose.position.y = pos[1];
        curr_pose.pose.position.z = 0.0;
        curr_pose.header = original_path.poses[optimized_path.poses.size()].header;
        optimized_path.poses.push_back(curr_pose);

  }
  _original_path_pub.publish(original_path);
  _optimized_path_pub.publish(optimized_path);

  if (_g2o_file.size() > 0) {
    _optimizer.save(_g2o_file.c_str());
  }

}

bool GroundTruthRefiner::exportPath(const std::string &filename) const {
    // Write to File
    std::ofstream ofs(filename.c_str(), std::ios::out|std::ios::binary);

    uint32_t serial_size = ros::serialization::serializationLength(optimized_path);
    boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);

    ros::serialization::OStream ostream(obuffer.get(), serial_size);
    ros::serialization::serialize(ostream, optimized_path);
    ofs.write((char*) obuffer.get(), serial_size);
    ofs.close();

    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "g2o_fiducial_slam");

    sleep(2); // Let the systems initialize
    std::unique_ptr<GroundTruthRefiner> node;
    node.reset(new GroundTruthRefiner);
    double rate = 4.0;
    

    int update_secs = 100;

    ros::NodeHandle pnh("~");

    int n_iter;
    pnh.param("update_rate", rate, 2.0);
    ros::Rate r(rate);
    pnh.param("n_iter", n_iter, 3);
    pnh.param("update_secs", update_secs, 60);
    std::string path_export_file;
    pnh.param("path_export_file", path_export_file, std::string("optimized_path"));
    const int update_iters = update_secs*rate;
    int cont = 0;


    
    while (ros::ok()) {
        ros::spinOnce();
        
        cont ++;

        // Check whether we have advanced or not and add a constraint
        node->updateOdometry();

        // Once the data has been collected --> optimize with g2o
        if (node != nullptr && cont % update_iters == 0) {
            node->optimize(n_iter);
        }
        r.sleep();
    }

    cout << "Experiment finished. Optimizing and exporting\n";

    node->optimize(n_iter+10);
    if (node->exportPath(path_export_file)) {
        ROS_INFO("Path exported to: %s", path_export_file.c_str());
    }

    return 0;
}


