#!/usr/bin/env python
import rospy
from tf_conversions import transformations
from numpy import zeros
from geometry_msgs.msg import TransformStamped, Quaternion, PoseWithCovarianceStamped, PoseStamped

class InitialPosePublisher:
    """
    Class for publishing the first GT Pose as the initial pose of AMCL, or any localization method
    """
    def __init__(self):
        self.init = False    #We did not receive any GT data yet
        if rospy.has_param("use_covariance_subscriber"):
            self.pose_sub = rospy.Subscriber("pose", PoseWithCovarianceStamped, callback=self.poseCovCb)
        else:
            self.pose_sub = rospy.Subscriber("pose", PoseStamped, callback=self.poseCb)
        self.pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, latch=True, queue_size=2)

        self.covariance = zeros(36)
        self.covariance[0] = 0.2
        self.covariance[7] = 0.2
        self.covariance[35] = 0.2

    # Each time a pose is received --> publish it as the initial pose for localization (if not initialized)
    def poseCb(self, data):
        if self.init == False:
            pose = PoseWithCovarianceStamped()
            
            pose.header = data.header
            pose.pose.pose = data.pose
            pose.pose.covariance = self.covariance
            self.init = True
            self.pose_pub.publish(pose)
            rospy.loginfo("Published initialpose")

    # Each time a pose is received --> publish it as the initial pose for localization (if not initialized)
    def poseCovCb(self, data):
        if self.init == False:
            self.init = True
            self.pose_pub.publish(data)

if __name__ == "__main__":
    rospy.init_node("initial_pose_publisher")
    node = InitialPosePublisher()
    rospy.spin()


