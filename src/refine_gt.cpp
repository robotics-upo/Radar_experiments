/*
 * Copyright (c) 2021, Service Robotics Lab
 * All rights reserved.
 *
 * See LICENSE file for distribution details
 */
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

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

// #include <smart_ptr/unique_ptr.hpp>

using namespace std;
using namespace g2o;

void changeTf(tf2::Transform &t, double x, double y, double theta)  {
    t.setOrigin(tf2::Vector3(x, y, 0.0));
    auto q = t.getRotation();
    tf2::Matrix3x3 M(q);
    double roll, pitch, yaw;
    M.getRPY(roll, pitch, yaw);
    q.setRPY(roll, pitch, theta);
    t.setRotation(q);
}


class GroundTruthRefiner {

    public:
    GroundTruthRefiner();
    

    void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);

    void updateOdom(); // Whenever the base displaces over a threshold --> put an extra odometry measure

    void optimize(int n_rounds);

    protected:
    // ROS Stuff
    ros::Subscriber _gt_sub;
    ros::Publisher _path_pub;
    tf2::Transform _last_tf_odom_base;
    tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> listener;

    // G2O STUFF
    
    // allocating the optimizer
    g2o::SparseOptimizer _optimizer;
    std::unique_ptr<SlamLinearSolver> _linear_solver; 
    Eigen::Matrix3d _odometry_information, _observation_information;
    std::string _baseFrame, _odomFrame, _mapFrame;

    const int _position_id_offset = 1000000;
    std::string _g2o_file;

    int _frame_num = 0;

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
    _path_pub = ros::Publisher(pnh.advertise<nav_msgs::Path>("/path", 100));

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

    // Frames
    nh.param<std::string>("map_frame", _odomFrame, "map");
    nh.param<std::string>("odom_frame", _odomFrame, "odom");
    nh.param<std::string>("base_frame", _baseFrame, "base_footprint");

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
    pnh.param<double>("rot_noise_obs", rot_noise, 0.03);
    pnh.param<double>("trans_noise_obs", trans_noise_x, 0.1);
    covariance(0, 0) = trans_noise_x*trans_noise_x;
    covariance(1, 1) = trans_noise_x*trans_noise_x; // NOTE: not a bug, same noise for x and y 
    covariance(2, 2) = rot_noise*rot_noise;
    _observation_information = covariance.inverse();

    // if (load(_g2o_file))
    //     ROS_INFO("Stats file %s loaded successfully", _g2o_file.c_str());

    ROS_INFO("G2O Fiducial Slam ready");

    // Add the first vertex at the origin (0,0,0) in order to be able to include estimated pose edges
    _frame_num = 0;
    const SE2 origin_pose(0.0, 0.0, 0.0);
    VertexSE2* origin = new VertexSE2;
    origin->setId(0);
    origin->setEstimate(origin_pose);
    _optimizer.addVertex(origin);

    _frame_num = 1;
}

void GroundTruthRefiner::groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr &pose) {
    tf2::Quaternion q;
    tf2::fromMsg(pose->pose.orientation, q);
    tf2::Matrix3x3 M(q);
    double roll,pitch, theta;
    M.getRPY(roll, pitch, theta);

    double x = pose->pose.position.x;
    double y = pose->pose.position.y;
    ROS_INFO("Received pose. Coords: (%f, %f, %f)", x, y, theta);
    
    
    const SE2 robot_pose(x, y, theta);
    VertexSE2* robot = new VertexSE2;
    robot->setId(_frame_num);
    robot->setEstimate(robot_pose);
    _optimizer.addVertex(robot);

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

    // Add observation constraint (between origin and the vertex)
    EdgeSE2* observation = new EdgeSE2;
    observation->vertices()[0] = _optimizer.vertex(_frame_num);
    observation->vertices()[1] = _optimizer.vertex(0);
    const SE2 obs_meas(x, y, theta);
    observation->setMeasurement(obs_meas);
    observation->setInformation(_observation_information);  // TODO: how to estimate?
    _optimizer.addEdge(observation);
    _frame_num++;
    _last_tf_odom_base = tf_odom_base;
}

/*********************************************************************************
   * optimization
   ********************************************************************************/
void GroundTruthRefiner::optimize(int n_rounds) {
  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
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
  int max_id = 0;

  // Initialize path for visualization 
  nav_msgs::Path p;
  static int path_seq = 0;
  p.header.frame_id = _mapFrame;
  p.header.stamp = ros::Time::now();
  p.header.seq = path_seq++;

  for (auto &x:_optimizer.vertices()) {
      
        auto y = dynamic_cast<VertexSE2 *> ( x.second);
        auto &pos = y->estimate();
        if (y == nullptr || y->id() == 0) continue;
        
        max_id = std::max(max_id, x.first); // Get max ID

        geometry_msgs::PoseStamped curr_pose;
        curr_pose.header = p.header;
        tf2::Transform t;
        changeTf(t, pos[0], pos[1], pos[2]);
        curr_pose.pose.orientation = tf2::toMsg(t.getRotation());
        curr_pose.pose.position.x = pos[0];
        curr_pose.pose.position.y = pos[1];
        curr_pose.pose.position.z = 0.0;
        p.poses.push_back(curr_pose);
      
  }
  auto y = dynamic_cast<VertexSE2 *>(_optimizer.vertices()[max_id]);
  auto pos = y->estimate();

//   publishMarkers();
  _path_pub.publish(p);

  if (_g2o_file.size() > 0) {
    _optimizer.save(_g2o_file.c_str());
  }
  
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "g2o_fiducial_slam");
    
    sleep(2); // Let the systems initialize
    std::unique_ptr<GroundTruthRefiner> node;
    node.reset(new GroundTruthRefiner);
    double rate = 20.0;
    ros::Rate r(rate);
    
    int update_secs = 100;

    ros::NodeHandle pnh("~");
    
    int n_iter;
    pnh.param("n_iter", n_iter, 3);
    pnh.param("update_secs", update_secs, 10);
    const int update_iters = update_secs*rate;
    int cont = 0;

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
        // node->publishMarkers();
        cont ++;

        // if (cont%100 == 0)
        //     node->publishMarkers();

        // Once the data has been collected --> optimize with g2o
        if (node != nullptr && cont % update_iters == 0) {
            node->optimize(n_iter);
        }
    }
   
    return 0;
}


