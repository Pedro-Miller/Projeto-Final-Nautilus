#!/usr/bin/python3

import rospy
import tf2_ros as tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Vector3, PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf.transformations as tft


# Global variables for object detection
green_detected = False
red_detected = False
red_direction = 0.0


# Callback for sonar data
def sonar_callback(msg, state):
    state['sonar_data'] = (msg.point.x, msg.point.y)


# Callback for odometry data
def odom_callback(msg, state):
    state['odom_data'] = msg.pose.pose


# Callback for image data
def image_callback(msg):
    global green_detected
    global red_detected
    global red_direction

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Convert image to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define color range for green
    green_low = np.array([40, 50, 50])
    green_high = np.array([80, 255, 255])

    # Define color range for red
    red_low1 = np.array([0, 70, 50])
    red_high1 = np.array([10, 255, 255])
    red_low2 = np.array([170, 70, 50])
    red_high2 = np.array([180, 255, 255])

    # Threshold the image for green and red colors
    green_mask = cv2.inRange(hsv, green_low, green_high)
    red_mask1 = cv2.inRange(hsv, red_low1, red_high1)
    red_mask2 = cv2.inRange(hsv, red_low2, red_high2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # Find contours in the green and red masks
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Reset detection flags
    green_detected = False
    red_detected = False
    id_detect = []
    # If a green object is detected, set green_detected flag
    if len(green_contours) > 0:
        green_detected = True
        id_detect[0] = red_detected
        id_detect[1] = green_detected
        return id_detect

    # If a red object is detected, and green is not, set red_detected flag
    elif len(red_contours) > 0 and not len(green_contours) > 0:
        red_detected = True
        id_detect[0] = red_detected
        id_detect[1] = green_detected
        return id_detect



# Function to calculate angular velocity required for orientation
def orientar(state):
    if 'sonar_data' in state and 'odom_data' in state:
        sonar_data = state['sonar_data']
        odom_data = state['odom_data']

        # Extract position and orientation data from the odom message
        px = odom_data.position.x
        py = odom_data.position.y
        orientation = odom_data.orientation

        # Convert orientation to Euler angles and extract yaw angle
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tft.euler_from_quaternion(quat)

        # Extract sonar position data
        x_m= sonar_data[0]
        y_m= -sonar_data[1]

        rospy.loginfo(str(x_m)+ " : " +str(y_m))
        # Calculate target angle using cross product of two vectors
        target_angle = math.atan2(x_m, y_m)
        aerror = target_angle - yaw

        rospy.loginfo('yaw:'+ str(yaw) +'tangl: ' + str(target_angle)+ ' aer: '+str(aerror) )



        # Normalize angle error to -pi to pi range
        while aerror > math.pi:
            aerror -= 2 * math.pi
        while aerror < -math.pi:
            aerror += 2 * math.pi



        rospy.loginfo('aer after norm: '+ str(aerror))
        

        # Calculate angular velocity required for orientation
        vel_ang = aerror

        return vel_ang


def control_speed(state):
    #importa variavel de estado do sonar
    if 'sonar_data' in state:
        sonar_data = state['sonar_data']
        # extrai posição do sonar
        x_m= sonar_data[0]
        y_m= -sonar_data[1]

        #calcula distancia
        dist = math.sqrt(x_m**2 + y_m**2)

        #define distancia min e max
        min_dist = 2.0
        max_dist = 5.0


        speed_fator = (dist - min_dist) / (max_dist - min_dist)
        speed_fator = min(1.0, max(0.0, speed_fator)) #map de 0 a 1


        max_speed = 2.0
        min_speed = 0.0

        speed = (max_speed - min_speed) * speed_fator + min_speed

        return speed

def andar(state, pub, rate):
            twist = Twist()

            # Calculate angular velocity for orientation
            vel_ang = orientar(state)

            # Set angular velocity in twist message
            if vel_ang is not None:
                twist.angular.z = vel_ang * 4

            # Set linear velocity in twist message = 
            speed = control_speed(state)
            if speed is not None:
                twist.linear.y = float(speed)
            else:
                twist.linear.y = 0.0
                
            # Publish twist message
            pub.publish(twist)

            rate.sleep()

def main():
    # Initialize node
    rospy.init_node("algo")

    # Create state dictionary
    state = {}

    # Subscribe to sonar and odometry topics
    rospy.Subscriber("/sonar_data", PointStamped, sonar_callback, state)
    rospy.Subscriber("/odom", Odometry, odom_callback, state)
    rospy.Subscriber("/camera/image_raw", image_callback, Image)

    # Setup publisher for velocity commands
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(30)

    # Main loop
    while not rospy.is_shutdown():
       andar(state, pub, rate)
        


if __name__ == "__main__":
    main()
