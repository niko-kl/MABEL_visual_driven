#region includes
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

#region init
print("Starting...")
try:
    ser = serial.Serial(port='/dev/ttyACM0', # ls /dev/tty* to find your port
            baudrate = 9600,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            timeout = 1)
except IOError:
    ser = serial.Serial(port='/dev/ttyACM1', # ls /dev/tty* to find your port
            baudrate = 9600,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            timeout = 1)
print("Initialized serial communication")

negative_byte = 0b1000000000000000
position_value_byte = 0b0100000000000000
angle_value_byte = 0b0010000000000000
#endregion

global secondStep

def checkDetection(data):
    detections = data.detections
    if len(detections) < 1: #später, nur weiter wenn mehr als 2 (= 1 mögliches Tor)
        #rotate(22)
        return
    tags = extractTagValues(detections)
    # print("distance_x = ", tags[0]['distance_x'])
    # print("yaw = ", tags[0]['yaw'], "\n")
    # zum testen: nur den ersten tag benutzen
    if not secondStep:
        ausrichten(tags)
    else:
        anfahren(tags)

def ausrichten(tags):
    distX = tags[0]['distance_x']
    distZ = tags[0]['distance_z']
    yaw = tags[0]['yaw']
    deltaAngle = 90 - abs(yaw)

    if deltaAngle > 80:
        if distX < 0:
            deltaAngle *= -1
    elif yaw > 0:
        deltaAngle *= -1
    deltaAngle = int(deltaAngle)
    sleep(2)
    rotate(deltaAngle)
    print("first rotation done, was ", deltaAngle)

    distancefirst = int(distZ * math.cos(np.radians(abs(deltaAngle))) * 100) - 15
    
    drive(distancefirst)
    print("first drive done, was ", distancefirst, "cm")

    if deltaAngle < 0:
        rotate(90)
    else:
        rotate(-90)
    print("second rotation done")

def anfahren(tags):
    distZ = tags[0]['distance_z']
    lastDistance = int(distZ * 100) - 10
    drive(lastDistance)
    print("last drive done, was ", lastDistance, "cm")

def rotate(angle):
    sendByte = 0b00000000

    if angle < 0:
        negative = True
        angle *= -1
        angle |= negative_byte
    angle |= angle_value_byte

    first = angle >> 8
    second = angle & 255

    val = pack("B", first)
    ser.write(val)
    sleep(0.05)
    val = pack("B", second)
    ser.write(val)

    result = int.from_bytes(ser.read(), "big")
    while result != 2:
        result = int.from_bytes(ser.read(), "big")
    return

def drive(distance):
    sendByte = 0b00000000

    if distance < 0:
        negative = True
        distance *= -1
        distance |= negative_byte
    distance |= position_value_byte

    first = distance >> 8
    second = distance & 255

    val = pack("B", first)
    ser.write(val)
    sleep(0.05)
    val = pack("B", second)
    ser.write(val)

    result = int.from_bytes(ser.read(), "big")
    while result != 1:
        result = int.from_bytes(ser.read(), "big")
    return

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
    print("Waiting 30 seconds before start")
    sleep(10)
    print("Arduino initialized")
    sleep(10)
    print("10 seconds left")
    sleep(10)
    rospy.init_node('mabel_visual_driven', anonymous=True)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, checkDetection, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    secondStep = False
    listener()