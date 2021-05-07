#region
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
#endregion

def checkDetection(data):
    detections = data.detections
    if not len(detections): #später, nur weiter wenn mehr als 2 (= 1 mögliches Tor)
        return
    tags = extractTagValues(detections)
    print("distance_x = ", tags[0]['distance_x'])
    print("yaw = ", tags[0]['yaw'], "\n")
    sleep(1)
    # zum testen: nur den ersten tag benutzen
    ausrichten(tags)

def ausrichten(tags):
    distX = tags[0]['distance_x']
    yaw = tags[0]['yaw']
    deltaAngle = 90 - abs(yaw)

    if deltaAngle > 80:
        if distX < 0:
            deltaAngle *= -1
    elif yaw > 0:
        deltaAngle *= -1

def extractTagValues(detections):
    tags = []
    for x in range(len(detections)):
        quaternion = (
            detections[x].pose.pose.pose.orientation.x, 
            detections[x].pose.pose.pose.orientation.y, 
            detections[x].pose.pose.pose.orientation.z, 
            detections[x].pose.pose.pose.orientation.w)

        pitch, yaw, roll = tf.transformations.euler_from_quaternion(quaternion)
        roll =  np.degrees(roll)
        pitch = np.degrees(pitch)
        yaw = np.degrees(yaw)
        if pitch < 0.0:
            pitch = pitch + 180.0
        elif pitch > 0.0:
            pitch = pitch - 180.0

        tags.append({
        'id': detections[x].id[0],
        'distance_x': detections[x].pose.pose.pose.position.x,
        'distance_y': detections[x].pose.pose.pose.position.y,
        'distance_z': detections[x].pose.pose.pose.position.z,
        'roll': roll, 
        'pitch': pitch, 
        'yaw': yaw, 
        })
    return tags

def listener():
    rospy.init_node('apriltag_listener', anonymous=True)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, checkDetection, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()