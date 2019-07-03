#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class Spike:
    
    def __init__(self):

        self.vis_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)

    def odom_cb(self, data):
        msg = PoseStamped()
        msg.header = data.header
        msg.pose = data.pose.pose
        self.vis_pub.publish(msg)
        
    def main(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('mavros_odom_spike', anonymous=True)

        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_cb)
        
        
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    spike = Spike()
    spike.main()
