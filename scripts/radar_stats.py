#!/usr/bin/env python
import rospy
from tf_conversions import transformations
from numpy import zeros
from sensor_msgs.msg import LaserScan

class LaserScanStats:
    """
    Class for publishing the first GT Pose as the initial pose of AMCL, or any localization method
    """
    def __init__(self):
        self.init = False    #We did not receive any GT data yet
        self.max_dist = rospy.get_param("~max_dist", 15.0)
        self.scan_sub = rospy.Subscriber("/lidar_radar_fusion_node/result_laser_scan", LaserScan, callback=self.scanCb)
        self.range_filename = rospy.get_param("~filename", "radar_stats.m")
        print "RADAR STATS. Opening file:", self.range_filename
        self.range_file = open(self.range_filename, 'w')

    # Each time a pose is received --> publish it as the initial pose for localization (if not initialized)
    def scanCb(self, data):
        cont = 0
        
        for i in data.ranges:
            if i < self.max_dist:
                cont+=1

        # data = LaserScan()
        # data.header.stamp.

        print data.header.stamp.to_time()
        self.range_file.write(str(data.header.stamp.to_time()))
        self.range_file.write(" ")
        self.range_file.write(str(cont))
        self.range_file.write("\n")

if __name__ == "__main__":
    rospy.init_node("laser_scan_stats")
    node = LaserScanStats()
    rospy.spin()
    node.range_file.close()


