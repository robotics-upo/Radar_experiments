#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from idmind_robot.msg import Log
from nav_msgs.msg import Odometry
from threading import Lock
from tf2_geometry_msgs import PoseStamped
from tf_conversions import transformations
from idmind_motorsboard.msg import WheelsMB
from idmind_sensorsboard.msg import SystemVoltages
from numpy import pi, sin, cos
from geometry_msgs.msg import TransformStamped, Quaternion
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

VERBOSE = 5
LOGS = 5


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
        self.name = rospy.get_param("/bot/name", default="bot")
        self.kinematics = rospy.get_param("/bot/kinematics", default="2wd")
        self.base_width = rospy.get_param("/bot/base_width", default=0.26)
        self.simulation = rospy.get_param("/simulation", default=False)
        self.control_freq = rospy.get_param("/move_base/controller_frequency", default=20.)
        self.use_imu = rospy.get_param("~use_imu", default=2)
        self.publish_tf = rospy.get_param("/publish_tf", default=True) # UPO: added to switch between mapping and navigation
        self.yaw = 0

        if not self.simulation:
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

            waiting = True
            # while waiting:
            #     try:
            #         self.log("Waiting for Sensors Board", 5)
            #         rospy.wait_for_message("/idmind_sensors/voltages", SystemVoltages, timeout=1)
            #         self.voltages = SystemVoltages()
            #         rospy.Subscriber("/idmind_sensors/voltages", SystemVoltages, self.handle_voltages)
            #         waiting = False
            #     except rospy.ROSException:
            #         self.log("Sensor Board not responding, waiting 2 secs", 1, alert="warn")
            #         rospy.sleep(2)

        else:
            self.simul_odom = Odometry()
            self.simul_odom.pose.pose.orientation.w = 1.0
            self.last_odom = Odometry()
            self.last_odom.pose.pose.orientation.w = 1.0
            rospy.wait_for_message("/gazebo/odom", Odometry)
            rospy.Subscriber("/gazebo/odom", Odometry, self.update_odometry)
            self.log("Connecting to Gazebo", 5)

        #######################
        #  Sensor Parameters  #
        #######################
        # Service for the calibration of IMU information
        self.imu_q_offset = Quaternion()
        self.imu_q_offset.w = 1.
        rospy.Service("/idmind_navigation/calibrate_imu", Trigger, self.calibrate_imu)
        rospy.Service("/idmind_navigation/display_calib", Trigger, self.display_calib)
        self.imu_reading = Imu()
	
        if not self.simulation:
            if self.use_imu == 1:
                rospy.Subscriber("/imu", Imu, self.update_imu, queue_size=1)
            elif self.use_imu == 2:
                rospy.Subscriber("/os1_cloud_node/imu", Imu, self.update_imu_gyroscope, queue_size=1)
        else:
            rospy.Subscriber("/gazebo/imu", Imu, self.update_imu)

        ##############
        #  Odometry  #
        ##############
        self.new_encoder = False
        self.odom_time = rospy.Time.now()
        self.last_imu_time = None
        # Current Odom state
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.roll = 0
        self.pitch = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
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

    #################################
    #  Callbacks and other updates  #
    #################################
    def report_ready(self, _req):
        """ Simple Service callback to show node is ready """
        return TriggerResponse(self.ready, "IDMind Navigation is " + ("ready" if self.ready else "not ready"))

    def handle_voltages(self, msg):
        """
        Callback to messages received in /idmind_sensors/voltages
        :return:
        """
        self.voltages = msg

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
        self.imu_reading = msg
    def update_imu_gyroscope(self, msg):
	
        if self.last_imu_time == None:
            self.last_imu_time = rospy.Time.now()
            self.theta = 0
            self.gyro_bias = msg.angular_velocity.z
            return
    
        curr_time = rospy.Time.now()
    
        self.theta += (msg.angular_velocity.z-self.gyro_bias) * (curr_time - self.last_imu_time).to_sec()
        self.last_imu_time = curr_time

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
        """
        This method will save the current orientation as the offset.
        All future publications will be adjusted in relation to the saved orientation
        :return:
        """
        self.log("Calibrating IMU", 5)
        r = rospy.Rate(10)
        calibrated = False
        while not calibrated and not rospy.is_shutdown():
            rospy.wait_for_message("/imu", Imu)
            try:
                q = self.imu_reading.orientation
                ###################
                # Simple rotation #
                ###################
                # self.imu_offset = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                #######################
                # Quaternion Rotation #
                #######################
                trans = self.tf_buffer.lookup_transform("base_link", "base_link_imu", rospy.Time.now(), rospy.Duration(1.0))
                imu_pose = PoseStamped()
                imu_pose.header.frame_id = "base_link_imu"
                imu_pose.pose.orientation = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
                imu_real = self.tf_buffer.transform(imu_pose, "base_link")
                self.imu_q_offset.x = imu_real.pose.orientation.x
                self.imu_q_offset.y = imu_real.pose.orientation.y
                self.imu_q_offset.z = imu_real.pose.orientation.z
                self.imu_q_offset.w = -imu_real.pose.orientation.w
                calibrated = True
            except KeyboardInterrupt:
                raise KeyboardInterrupt()
            except LookupError as l_error:
                self.log("Lookup error in IMU calibration: {}".format(l_error), 5)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.log("tf2 exception")
            r.sleep()
        self.log("IMU Calibrated", 5)
        return TriggerResponse(True, "Calibration completed")

    def display_calib(self, _req):
        return TriggerResponse(True, "{}".format(self.imu_q_offset))

    ##############
    #  Odometry  #
    ##############
    def broadcast_odometry(self):
        """
        Broadcasts the odometry based in tick readings and imu readings, if available
        Currently supports only 2wd
        :return:
        """

        # Check if IMU readings can be used. If yes, dth is set
        try:
            if self.use_imu == 1:
                if (rospy.Time.now() - self.imu_reading.header.stamp).to_sec() < 1.:
                    # q = self.imu_reading.orientation
                    ###################
                    # Simple rotation #
                    ###################
                    # imu_val = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2] - self.imu_offset
                    #######################
                    # Quaternion Rotation #
                    #######################
                    # Get the IMU reading and apply the calibration quaternion, while in imu frame
                    last_imu = self.imu_reading.orientation
                    imu_raw_val = transformations.euler_from_quaternion([last_imu.x,
                                                                         last_imu.y,
                                                                         last_imu.z,
                                                                         last_imu.w])[2]

                    # Transform the imu reading to base_link frame
                    imu_pose = PoseStamped()
                    imu_pose.header.frame_id = "base_link_imu"
                    imu_pose.pose.orientation = Quaternion(x=last_imu.x, y=last_imu.y, z=last_imu.z, w=last_imu.w)
                    imu_real = self.tf_buffer.transform(imu_pose, "base_link")
                    imu_pre_val = transformations.euler_from_quaternion([imu_real.pose.orientation.x,
                                                                         imu_real.pose.orientation.y,
                                                                         imu_real.pose.orientation.z,
                                                                         imu_real.pose.orientation.w])[2]

                    # Apply the calibration to the imu reading
                    q_base = imu_real.pose.orientation
                    q_off = self.imu_q_offset
                    q_corrected = transformations.quaternion_multiply(
                        [q_base.x, q_base.y, q_base.z, q_base.w],
                        [q_off.x, q_off.y, q_off.z, q_off.w])

                    # Extract the yaw value (z axis first, since its the main use
                    imu_val = transformations.euler_from_quaternion(
                        [q_corrected[0], q_corrected[1], q_corrected[2], q_corrected[3]])[2]
                    self.log("Raw: {} | Rot: {} | Corr {}".format(round(imu_raw_val, 4), round(imu_pre_val, 4),
                                                                  round(imu_val, 4)), 7)
                    dth = imu_val - self.th
                    use_imu = True
                else:
                    dth = 0
                    use_imu = False
            elif self.use_imu == 2:
                dth = self.theta - self.th
                a_x = self.imu_reading.linear_acceleration.x
                a_y = self.imu_reading.linear_acceleration.y
                a_z = self.imu_reading.linear_acceleration.z

                pitch = atan(a_x/sqrt(a_y*a_y + a_z*a_z) )
                roll =  atan(a_y/sqrt(a_x*a_x + a_z*a_z) )
                use_imu = True
        except Exception as imu_err:
            self.log("Error in reading IMU messages: {}".format(imu_err), 3, alert="warn")
            use_imu = False
            dth = 0

        if self.simulation:
            ##############
            # SIMULATION #
            ##############
            self.log("Broadcasting odometry for simulation", 7)
            try:
                last_time = self.odom_time
                self.odom_time = rospy.Time.now()
                curr_odom = Odometry(header=self.simul_odom.header, child_frame_id=self.simul_odom.child_frame_id,
                                     pose=self.simul_odom.pose, twist=self.simul_odom.twist)

                dx = curr_odom.pose.pose.position.x - self.last_odom.pose.pose.position.x
                dy = curr_odom.pose.pose.position.y - self.last_odom.pose.pose.position.y

                # If IMU is not usable, use simulation odometry readings
                if not use_imu:
                    curr_th = transformations.euler_from_quaternion(
                        [curr_odom.pose.pose.orientation.x, curr_odom.pose.pose.orientation.y,
                         curr_odom.pose.pose.orientation.z, curr_odom.pose.pose.orientation.w])[2]
                    last_th = transformations.euler_from_quaternion(
                        [self.last_odom.pose.pose.orientation.x, self.last_odom.pose.pose.orientation.y,
                         self.last_odom.pose.pose.orientation.z, self.last_odom.pose.pose.orientation.w])[2]
                    dth = curr_th - last_th

                self.last_odom = curr_odom

            except Exception as simul_err:
                rospy.logerr("{}: Exception computing odometry in simulation - {}".format(rospy.get_name(), simul_err))
                return
        else:
            ###############
            #    ROBOT    #
            ###############
            self.log("Broadcasting odometry for Robot", 7)
            last_time = self.odom_time
            self.odom_time = rospy.Time.now()

            try:
                # Start by computing the linear and angular displacement
                if self.kinematics == "2wd":

                    # Get state changes
                    self.wheels_lock.acquire()
                    dleft = self.wheel_odom["front_left"]
                    dright = self.wheel_odom["front_right"]
                    self.wheel_odom["front_left"] = 0.
                    self.wheel_odom["front_right"] = 0.
                    self.wheels_lock.release()

                    dlinear_x = (dright - dleft) / 2 * cos(self.roll)
                    dlinear_y = 0
                    dlinear_z =  (dright - dleft) / 2 * sin(self.roll)
                    # If IMU is not usable, use simulation odometry readings
                    dth = (dright + dleft) / self.base_width

                if dth > pi:
                    dth = dth - 2 * pi
                if dth < - pi:
                    dth = dth + 2 * pi

                #if self.th > pi:
                #    self.th = self.th - 2 * pi
                #if self.th < - pi:
                #    self.th = self.th + 2 * pi

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
            odom_msg.pose.pose.orientation = Quaternion(*q)
            odom_msg.twist.twist.linear.x = self.vx
            odom_msg.twist.twist.linear.y = self.vy
            odom_msg.twist.twist.angular.z = self.vth

            self.odom_pub.publish(odom_msg)
        except Exception as err:
            rospy.logfatal("{}: Exception in BroadCast Odometry: {}".format(rospy.get_name(), err))
            return

    def start(self):
        r = rospy.Rate(self.control_freq)

        if self.use_imu == 1:
        	self.calibrate_imu(TriggerRequest())
        while not rospy.is_shutdown():
            try:
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
