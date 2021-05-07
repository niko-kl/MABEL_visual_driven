#!/usr/bin/env python
import rospy
from std_msgs import msg
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetection
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Point
import serial
from struct import pack
from time import sleep
import math
import tf.transformations
import numpy as np

def callback(data):
    print("Hallo Welt")

def listener():
    rospy.init_node('apriltag_listener', anonymous=True)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()