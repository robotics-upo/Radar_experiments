#!/usr/bin/env python

import rospy
from tf2_geometry_msgs import PoseStamped
from tf_conversions import transformations
from numpy import pi, sin, cos
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import Buffer, TransformListener

class GroundTruth:
    """
    Class for comparing GT data (from fiducials) with tf data
    """
    def __init__(self):
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer)
        self.pose_sub = rospy.Subscriber("/fiducial_slam/pose", PoseStamped, callback=self.poseCb)
        self.filename = rospy.get_param("~/filename", default="ground_truth_stats.txt")
        self.base_frame = rospy.get_param("~/base_frame", default="base_link")
        self.map_frame = rospy.get_param("~/map_frame", default="map")
        try:
            self.stats_file = open(self.filename, 'w')
        except OSError as err:
            print('Could not open file %s'%format(self.filename))
        

    # Each time a pose is received --> save its coords (GT) with the tf 
    # Format of the file: t x y yaw x_gt y_gt yaw_gt
    def poseCb(self, data):
        try: 
            self.t = self.tf_buffer.lookup_transform(self.map_frame, 
                                                     self.base_frame,
                                                     rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        (roll,pitch, yaw) = transformations.euler_from_quaternion(self.t.transform.rotation)
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        (roll,pitch, yaw2) = transformations.euler_from_quaternion(quaternion)
        self.stats_file.write('%f %f %f %f %f'.format(data.header.stamp, self.t.transform.translation.x, self.t.transform.translation.y, yaw,
                                                    data.pose.position.x, data.pose.position.y, yaw2))
        



if __name__ == "__main__":
    rospy.init_node("ground_truth")
    node = GroundTruth()
    rospy.spin()


