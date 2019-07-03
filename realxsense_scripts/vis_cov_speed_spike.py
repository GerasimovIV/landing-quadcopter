#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan


class Spike:
    
    def __init__(self):

        self.vis_pub = rospy.Publisher("/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, queue_size=10)
        self.vis_speed_pub = rospy.Publisher("/mavros/vision_speed/speed_twist_cov", TwistWithCovarianceStamped, queue_size=10)
        self.odom_pub = rospy.Publisher("/odometry", Odometry, queue_size=10)
        self.alt = 0.

    def fodom_cb(self, data):
        msg = PoseWithCovarianceStamped()
        msg.header = data.header
        msg.pose.pose = data.pose.pose
        msg.pose.covariance = data.pose.covariance
        self.vis_pub.publish(msg)
        
        spmsg = TwistWithCovarianceStamped()
        spmsg.header = data.header
        spmsg.twist = data.twist
        self.vis_speed_pub.publish(spmsg)
        
    def odom_cb(self, data):
        odmsg = data
        #if self.alt != 0.:
        #    odmsg.pose.pose.position.z = self.alt - 0.1
        self.odom_pub.publish(odmsg)
        
    def alt_cb(self, data):
        alt = data.ranges[0]
        if alt > data.range_min and alt < data.range_max:
            self.alt = alt
        else:
            self.alt = 0.
        
        
    def main(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('mavros_odom_spike', anonymous=True)

        rospy.Subscriber("/odometry/filtered", Odometry, self.fodom_cb)
        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/sf30/range", LaserScan, self.alt_cb)
        
        
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    spike = Spike()
    spike.main()
