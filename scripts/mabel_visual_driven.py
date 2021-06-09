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
from time import sleep
from struct import pack
import tf.transformations
import numpy as np
import math
from approxeng.input.selectbinder import ControllerResource
#endregion

mode = ''
gates = [   {'ids': [0, 1], 'found': 0, 'firstTagIndex': -1, 'secondTagIndex': -1},
            {'ids': [2, 3], 'found': 0, 'firstTagIndex': -1, 'secondTagIndex': -1}]

negative_byte = 0b1000000000000000
position_value_byte = 0b0100000000000000
angle_value_byte = 0b0010000000000000

def rotate(angle):
    global ser
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
    global ser
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

def getTagValues(detections):
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

def resetGates():
    for x in range(len(gates)):
        gates[x]['found'] = 0

def getGateValues(tags):
    resetGates()
    resultValues = []
    for x in range(len(tags)):
        for y in range(len(gates)):
            if tags[x]['id'] in gates[y]['ids']:
                if gates[y]['firstTagIndex'] == -1:
                    gates[y]['firstTagIndex'] = tags[x]['id']
                gates[y]['secondTagIndex'] = tags[x]['id']
                gates[y]['found'] += 1
    
    gateIndex = -1
    for x in range(len(gates)):
        if gates[x]['found'] == 2:
            gateIndex = x
            break
    
    if gateIndex >= 0:
        finalTags = []
        finalTags.append(tags[gates[gateIndex]['firstTagIndex']])
        finalTags.append(tags[gates[gateIndex]['secondTagIndex']])

        distY = int(((finalTags[0]['distance_x'] + finalTags[1]['distance_x']) / 2) * 100)
        distX = int(((finalTags[0]['distance_z'] + finalTags[1]['distance_z']) / 2) * 100)
        yaw = int((finalTags[0]['yaw'] + finalTags[1]['yaw']) / 2)

        resultValues.append({'distY': distY, 'distX': distX, 'yaw': yaw})

    return resultValues

def evaluateDetection(data):
    if getControlMode() == 'circle':
        return
    tags = getTagValues(data.detections)
    gateValues = getGateValues(tags)
    if gateValues:
        print("Found gate:")
        print(gateValues)
    else:
        print("No gates were found")

    choice = input("Enter mode: (d) drive 30 cm or (r) rotate 90 degrees: ")
    if choice == "d":
        drive(30)
    elif choice == "r":
        rotate(90)
    else:
        print("sleeping 3 seconds")
        sleep(3)
    # if len(tags) < 2:
    #     rotate(30)
    #     return

def getControlMode():
    global joystick
    joystick.check_presses()
    presses = joystick.presses

    if 'square' in presses:
        return 'square'
    elif 'circle' in presses:
        return 'circle'

    return ''

def main():
    global ser
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

    rospy.init_node('mabel_visual_driven', anonymous=True)

    global sub
    global mode
    
    while True:
        try:
            global joystick
            with ControllerResource() as joystick:
                print('Found a joystick and connected')
                while joystick.connected:
                    if not mode:
                        mode = getControlMode()
                    while mode == 'circle':
                        sendByte = 0b00000000
                        left_x = joystick['lx']
                        left_y = joystick['ly']
                        if not(left_x == 0 and left_y == 0):
                            if left_x < 0:
                                print("Moving left")
                                sendByte |= 0b00000001
                            if left_x > 0:
                                print("Moving right")
                                sendByte |= 0b00000010
                            if left_y > 0:
                                print("Moving forward")
                                sendByte |= 0b00000100
                            if left_y < 0:
                                print("Moving backwards")
                                sendByte |= 0b00001000
                            val = pack("B", sendByte)
                            # ser.write(val)
                        sleep(0.05)

                        if getControlMode() == 'square':
                            mode = 'square'

                    if mode == 'square':
                        sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, evaluateDetection, queue_size=1)

                    while mode == 'square' and not rospy.core.is_shutdown():
                        rospy.rostime.wallsleep(0.5)
                        if getControlMode() == 'circle':
                            sub.unregister()
                            mode = 'circle'
                        
            # Joystick disconnected...
            print('Connection to joystick lost')
        except IOError:
            # No joystick found, wait for a bit before trying again
            print('Unable to find any joysticks')
            sleep(1.0)

if __name__ == '__main__':
    main()