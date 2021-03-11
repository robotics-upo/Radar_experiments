#!/usr/bin/env python

import rospy
from tf2_geometry_msgs import PoseStamped
from tf_conversions import transformations
from numpy import pi, sin, cos, sqrt
from geometry_msgs.msg import TransformStamped, Quaternion, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
import tf2_ros

class GroundTruth:
    """
    Class for comparing GT data (from fiducials) with tf data
    """
    def __init__(self):
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer)
        self.pose_sub = rospy.Subscriber("pose", PoseWithCovarianceStamped, callback=self.poseCb)
        self.filename = rospy.get_param("~filename", default="ground_truth_stats.txt")
        self.base_frame = rospy.get_param("~base_frame", default="base_link")
        self.map_frame = rospy.get_param("~map_frame", default="map")
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

        q = self.t.transform.rotation
        (roll,pitch, yaw_amcl) = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        p_amcl = self.t.transform.translation
        p_fid = data.pose.pose.position
        d1 = sqrt((p_amcl.x - p_fid.x)**2 + (p_amcl.y - p_fid.y)**2)

        q_fid = data.pose.pose.orientation
        (roll,pitch, yaw_fid) = transformations.euler_from_quaternion((q_fid.x, q_fid.y, q_fid.z, q_fid.w))

        yaw_dist = abs(yaw_amcl-yaw_fid)
        
        text = '{0} {1} {2} {3} {4} {5} {6} {7} {8}'.format(data.header.stamp, p_amcl.x, p_amcl.y, yaw_amcl,
                                                    p_fid.x, p_fid.y, yaw_fid, d1, yaw_dist)
        print text
        self.stats_file.write(text)
        self.stats_file.write('\n')
        



if __name__ == "__main__":
    rospy.init_node("ground_truth")
    node = GroundTruth()
    rospy.spin()


