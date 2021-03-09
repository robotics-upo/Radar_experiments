#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from idmind_robot.msg import Log
from nav_msgs.msg import Odometry
from threading import Lock
from tf2_geometry_msgs import PoseStamped
from tf_conversions import transformations
from idmind_motorsboard.msg import WheelsMB
from numpy import pi, sin, cos, sqrt
from geometry_msgs.msg import TransformStamped, Quaternion
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest, Empty, EmptyResponse
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from filterpy.kalman import ExtendedKalmanFilter as EKF
import simpy
from numpy import eye, array, asarray
import numpy as np
import tf2_ros

VERBOSE = 5
LOGS = 5

### Calculating Jacobians

def HJacobian_at(x):
    """ compute Jacobian of H matrix at x """
    rx = x[0]
    ry = x[1]
    
    crx = cos(rx)
    cry = cos(ry)
    srx = sin(rx)
    sry = sin(ry)
    
    return array ([[       0,     -cry,   0, 0, 0, 0], 
                   [crx*cry, -srx*sry,   0, 0, 0, 0],
                   [-crx*sry,   -crx*sry,  0, 0, 0, 0]])

    # return array ([[       0,     cry,   0, 0, 0, 0], 
    #                [-crx*cry, srx*sry,   0, 0, 0, 0],
    #                [crx*sry,   crx*sry,  0, 0, 0, 0]])
    
    
""" compute measurement for slant range that
would correspond to state x.
"""
def hx(x):
    return array([-sin(x[1]),sin(x[0])*cos(x[1]), cos(x[0])*cos(x[1])])


class IDMindOdometry:
    """
    Class responsible for simple navigation functions:
        - Publish odometry, either from Gazebo or calculating from motor ticks and IMU readings
        - Convert Twist messages from different controllers to motor velocities or to Gazebo
    """
    def __init__(self):

        ###############
        #   Logging   #
        ###############
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)

        ######################
        #  Robot Parameters  #
        ######################
        self.control_freq = rospy.get_param("/move_base/controller_frequency", default=20.)
        self.publish_tf = rospy.get_param("/publish_tf", default=True) # UPO: added to switch between mapping and navigation
     
        
        # EKF related stuff
        self.ekf_init = False # We have to wait for some imu measures before initialization
        self.calib_data = []
        self.calib_gx = []
        self.calib_gy = []
        self.calib_gz = []
        self.min_calib_size = rospy.get_param('~min_calib_size', default = 200)
        
        self.acc_dev = rospy.get_param("~acc_dev", default=0.1)
        self.gyr_dev = rospy.get_param("~gyr_dev", default=0.005)
        self.bias_dev = rospy.get_param("~bias_dev", default=0.0001)
        self.bias_th = rospy.get_param("~bias_thres", default=0.01)
        
        self.last_imu_time = None
     
        self.imu_bias = None
        
        #########################
        #  Hardware connection  #
        #########################
        waiting = True
        while waiting:
            try:
                self.log("Waiting for MotorBoard", 5)
                rospy.wait_for_message("/idmind_motors/wheel_odom", WheelsMB, timeout=1)
                self.wheels_lock = Lock()
                self.wheel_odom = {"front_right": 0., "front_left": 0., "back_right": 0., "back_left": 0.}
                rospy.Subscriber("/idmind_motors/wheel_odom", WheelsMB, self.handle_wheel_odom)
                waiting = False
            except rospy.ROSException:
                self.log("Motor Board not responding, waiting 2 secs", 1, alert="warn")
                rospy.sleep(2)

        #######################
        #  Sensor Parameters  #
        #######################
        # Service for the calibration of IMU information
        self.imu_q_offset = Quaternion()
        self.imu_q_offset.w = 1.
        rospy.Service("/idmind_navigation/calibrate_imu", Empty, self.calibrate_imu)
        rospy.Service("/idmind_navigation/display_calib", Trigger, self.display_calib)
        self.imu_reading = Imu()
        rospy.Subscriber("/os1_cloud_node/imu", Imu, self.update_imu, queue_size=1)
        
        ##############
        #  Odometry  #
        ##############
        self.new_encoder = False
        self.odom_time = rospy.Time.now()

        # Current Odom state
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.theta = 0
        self.roll = 0
        self.pitch = 0
        self.vx = 0
        self.vy = 0
        self.vth = 0

        # Publish odometry and broadcast odometry
        self.odom_broadcast = TransformBroadcaster()
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer)


        ###############################################
        #  Relevant External sensors (bumpers, doors) #
        ###############################################

        ###################
        #  Init Complete  #
        ###################
        self.ready = True
        rospy.Service("/idmind_navigation/ready", Trigger, self.report_ready)
        self.log("Node initialized", 5)
        
    def initialize(self, msg):
		# Do we have enought data for IMU initilaization ?
        self.calib_data.append(msg)
        self.calib_gx.append(msg.angular_velocity.x)
        self.calib_gy.append(msg.angular_velocity.y)
        self.calib_gz.append(msg.angular_velocity.z)
        
        if (len(self.calib_data) < self.min_calib_size):
            return False
		
		# Compute mean value and mean square
        self.gx_m = np.mean(asarray(self.calib_gx))
        self.gy_m = np.mean(asarray(self.calib_gy))
        self.gz_m = np.mean(asarray(self.calib_gz))
        self.gx_d = np.std(asarray(self.calib_gx))
        self.gy_d = np.std(asarray(self.calib_gx))
        self.gz_d = np.std(asarray(self.calib_gx))
        
        #  Chur's EKF initialization
        # The state is the angular variables and the gyro bias. 
        # A Observ. is the accelerometer reading.
        # The action is the gyro reading 
        self.ekf = EKF(dim_x=6, dim_u=3, dim_z=3) 
        dt = self.dt
        # Matrices of the system
        self.ekf.F = array([[1, 0, 0, -dt, 0, 0],
                            [0, 1, 0, 0, -dt, 0],
                            [0, 0, 1, 0, 0, -dt],
                            [0, 0, 0, 1, 0, 0  ],
                            [0, 0, 0, 0, 1, 0  ],
                            [0, 0, 0, 0, 0, 1  ]]) 
        self.ekf.B = array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1],
                            [0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0]])*dt
        # Initial covariance
        self.ekf.P = array([[pi/2, 0, 0, 0, 0, 0],
                            [0, pi/2, 0, 0, 0, 0],
                            [0, 0, pi/2, 0, 0, 0],
                            [0, 0, 0, dt**2, 0, 0  ],
                            [0, 0, 0, 0, dt**2, 0  ],
                            [0, 0, 0, 0, 0, dt**2  ]])

        # Measurement noise jac
        self.ekf.R = eye(3) * (self.acc_dev ** 2)
        
		# Initialize sensor variances
        gyr_dev = self.gyr_dev
        bias_dev = self.bias_dev
        
        # Process noise
        self.ekf.Q = array([[gyr_dev**2 * dt**2, 0, 0, 0, 0, 0],
                            [0, gyr_dev**2 * dt**2, 0, 0, 0, 0],
                            [0, 0, gyr_dev**2 * dt**2, 0, 0, 0],
                            [0, 0, 0, bias_dev**2 * dt, 0, 0  ],
                            [0, 0, 0, 0, bias_dev**2 * dt, 0  ],
                            [0, 0, 0, 0, 0, bias_dev**2 * dt  ]])
        
		
		# Initialize state vector x = [rx, ry, rz, gbx, gby, gbz]
        self.ekf.x = array([0, 0, self.theta, self.gx_m, self.gy_m, self.gz_m])
		
		# Check if calibration is good enough
        if self.gx_d < self.bias_th and self.gy_d < self.bias_th and self.gz_d < self.bias_th: 
            return True
        else: 
            # print (self.gx_d )
            return False


    #################################
    #  Callbacks and other updates  #
    #################################
    def report_ready(self, _req):
        """ Simple Service callback to show node is ready """
        return TriggerResponse(self.ready, "IDMind Navigation is " + ("ready" if self.ready else "not ready"))

    def handle_wheel_odom(self, msg):
        """
        Callback to messages received in /idmind_motors/wheel_odom
        :return:
        """
        try:
            self.wheels_lock.acquire()
            self.wheel_odom["front_right"] = self.wheel_odom["front_right"] + msg.front_right
            self.wheel_odom["front_left"] = self.wheel_odom["front_left"] + msg.front_left
            self.wheel_odom["back_right"] = self.wheel_odom["back_right"] + msg.back_right
            self.wheel_odom["back_left"] = self.wheel_odom["back_left"] + msg.back_left
            self.wheels_lock.release()
        except AttributeError as a_err:
            self.log("Lock is probably not defined yet: {}".format(a_err), 7)

    def update_odometry(self, msg):
        # Consider adding noise
        self.simul_odom = msg

    def update_imu(self, msg):
        if self.last_imu_time == None:
            self.last_imu_time = rospy.Time.now()
            return
        self.dt = (rospy.Time.now() - self.last_imu_time).to_sec()
        self.last_imu_time = rospy.Time.now()
        
        self.imu_reading = msg
        if not (self.ekf_init):
            self.ekf_init = self.initialize(msg)
        else:
            # Perform EKF update and predict stages
            # self.log("Performing EKF update and predict", 1)
            a_x = msg.linear_acceleration.x
            a_y = msg.linear_acceleration.y
            a_z = msg.linear_acceleration.z
            a = sqrt(a_x**2 + a_y**2 + a_z**2)
            a_x = a_x / a
            a_y = a_y / a
            a_z = a_z / a
            z = array([a_x, a_y, a_z])
            
            
            u_gyro = array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            
            self.ekf.predict(u = u_gyro)
            self.ekf.update(z, HJacobian_at, hx)
            
            # Take the estimated angles from the filter
            self.roll = self.ekf.x[0]
            self.pitch = self.ekf.x[1]
            self.theta = self.ekf.x[2]
            self.imu_bias = self.ekf.x[3:5]
            # print (self.roll, self.pitch, self.theta)
            print (z-hx(self.ekf.x))
            
    def log(self, msg, msg_level, log_level=-1, alert="info"):
        if VERBOSE >= msg_level:
            if alert == "info":
                rospy.loginfo("{}: {}".format(rospy.get_name(), msg))
            elif alert == "warn":
                rospy.logwarn("{}: {}".format(rospy.get_name(), msg))
            elif alert == "error":
                rospy.logerr("{}: {}".format(rospy.get_name(), msg))
        if LOGS >= (log_level if log_level != -1 else msg_level):
            self.logging.publish(rospy.Time.now().to_sec(), rospy.get_name(), msg)

    def calibrate_imu(self, _req):
        self.calib_data = []
        self.calib_gx = []
        self.calib_gy = []
        self.calib_gz = []
        self.ekf_init = False
        self.last_imu_time = None
        return EmptyResponse()
        
    def display_calib(self, _req):
         return TriggerResponse(True, "{}".format(self.imu_bias))

    ##############
    #  Odometry  #
    ##############
    def broadcast_odometry(self):
        """
        Broadcasts the odometry based in tick readings and imu readings, if available
        Currently supports only 2wd
        :return:
        """

        # Get the yaw displacement
        dth = self.theta - self.th
 	      

        ###############
        #    ROBOT    #
        ###############
        self.log("Broadcasting odometry for Robot", 7)
        last_time = self.odom_time
        self.odom_time = rospy.Time.now()

        try:
            # Start by computing the linear and angular displacement

            # Get state changes
            self.wheels_lock.acquire()
            dleft = self.wheel_odom["front_left"]
            dright = self.wheel_odom["front_right"]
            self.wheel_odom["front_left"] = 0.
            self.wheel_odom["front_right"] = 0.
            self.wheels_lock.release()

            dlinear_x = (dright - dleft) / 2 * cos(self.pitch)
            dlinear_y = 0
            dlinear_z =  -(dright - dleft) / 2 * sin(self.pitch)

            if dth > pi:
                dth = dth - 2 * pi
            if dth < - pi:
                dth = dth + 2 * pi

            dx = dlinear_x * cos(self.th + dth) - dlinear_y * sin(self.th + dth)
            dy = dlinear_x * sin(self.th + dth) + dlinear_y * cos(self.th + dth)
            dz = dlinear_z 

        except Exception as err:
            rospy.logfatal("{}: Exception computing odometry in real robot: {}".format(rospy.get_name(), err))
            return

        # Based on dx, dy and dth, compute, publish and broadcast odometry
        try:
            # Compute odometry positions
            self.x = self.x + dx
            self.y = self.y + dy
            self.z = self.z + dz
            self.th = self.th + dth
            
            self.vx = dx / (self.odom_time - last_time).to_sec()
            self.vy = dy / (self.odom_time - last_time).to_sec()
            self.vz = dz / (self.odom_time - last_time).to_sec()
            self.vth = dth / (self.odom_time - last_time).to_sec()

            # Build Transform message for Broadcast
            transf_msg = TransformStamped()
            transf_msg.header.stamp = self.odom_time
            transf_msg.header.frame_id = "odom"
            transf_msg.child_frame_id = "base_link"

            transf_msg.transform.translation.x = self.x
            transf_msg.transform.translation.y = self.y
            transf_msg.transform.translation.z = self.z

            q = transformations.quaternion_from_euler(self.roll, self.pitch, self.th)
            transf_msg.transform.rotation = Quaternion(*q)
            if self.publish_tf:
                self.odom_broadcast.sendTransform(transf_msg)

            # Build Odometry Message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.odom_time
            odom_msg.header.frame_id = "odom"

            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = self.z
            odom_msg.pose.pose.orientation = Quaternion(*q)
            odom_msg.twist.twist.linear.x = self.vx
            odom_msg.twist.twist.linear.y = self.vy
            odom_msg.twist.twist.linear.z = self.vz
            # odom_msg.twist.twist.angular.x = self.vth
            odom_msg.twist.twist.angular.z = self.vth

            self.odom_pub.publish(odom_msg)
        except Exception as err:
            rospy.logfatal("{}: Exception in BroadCast Odometry: {}".format(rospy.get_name(), err))
            return
        self.th = self.theta

    def start(self):
        r = rospy.Rate(self.control_freq)

        while not rospy.is_shutdown():
            try:
                if self.ekf_init:
                    self.broadcast_odometry()
                r.sleep()
            except KeyboardInterrupt:
                break
            except Exception as err:
                rospy.logerr(rospy.get_name()+" in start():" + str(err))

        rospy.logwarn(rospy.get_name()+": Shutting down")


if __name__ == "__main__":
    rospy.init_node("odometry_node")
    odom = IDMindOdometry()
    odom.start()
