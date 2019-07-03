#!/usr/bin/env python

import math
import rospy
import signal
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from functools import partial

def exit_signal(spike, signum, frame):
        signal.signal(signal.SIGINT, original_sigint)
        
        print("MAX IMU DIFF = " + str(max(spike.imu_dts)*1e-6))
        print("MIN IMU DIFF = " + str(min(spike.imu_dts)*1e-6))
        print("AVG IMU DIFF = " + str(sum(spike.imu_dts)*1e-6/len(spike.imu_dts)))
        
        print("MAX POSE DIFF = " + str(max(spike.pose_dts)*1e-6))
        print("MIN POSE DIFF = " + str(min(spike.pose_dts)*1e-6))
        print("AVG POSE DIFF = " + str(sum(spike.pose_dts)*1e-6/len(spike.pose_dts)))
        
        print("MAX COV = " + str(max(spike.covs)))
        #print("MIN COV = " + str(min(spike.covs)))
        
        sys.exit(1)

class Spike:
    
    def __init__(self):

        self.odom_t = 0
        self.pose_t = 0
        self.imu_t = 0
        self.imu_dts = []
        self.pose_dts = []
        self.covs = []

    def odom_cb(self, data):
        self.odom_t = data.header.stamp.secs * 1e9 + data.header.stamp.nsecs
        self.covs.extend(data.pose.covariance)
        self.print_diff("odom")
        
    def pose_cb(self, data):
        self.pose_t = data.header.stamp.secs * 1e9 + data.header.stamp.nsecs
        #self.print_diff("pose")
        
    def imu_cb(self, data):
        self.imu_t = data.header.stamp.secs * 1e9 + data.header.stamp.nsecs
        #self.print_diff("imu")
        
    def print_diff(self, imu_or_pose):
        if self.odom_t != 0:
            if imu_or_pose == "imu" and self.imu_t != 0:
                imu_dt = self.imu_t - self.odom_t
                #print("IMU DIFF = " + str(imu_dt))
                #self.imu_dts.append(imu_dt)
            elif imu_or_pose == "pose" and self.pose_t != 0:
                pose_dt = self.pose_t - self.odom_t
                #print("POSE DIFF = " + str(pose_dt))
                #self.pose_dts.append(pose_dt)
            elif imu_or_pose == "odom" and self.imu_t != 0 and self.pose_t != 0:
                imu_dt = self.imu_t - self.odom_t
                pose_dt = self.pose_t - self.odom_t
                #print("IMU DIFF = " + str(imu_dt))
                #print("POSE DIFF = " + str(pose_dt))
                self.imu_dts.append(imu_dt)
                self.pose_dts.append(pose_dt)
        
    def main(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('print_time_diff_node', anonymous=True)

        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        
        print("Node started")
        
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    #signal.signal(signal.SIGINT, exit_signal)
    spike = Spike()
    signal.signal(signal.SIGINT, partial(exit_signal, spike))
    spike.main()
