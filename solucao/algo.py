#!/usr/bin/python3

import rospy
import tf2_ros as tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Vector3, PointStamped
import math
import tf.transformations as tft



# Callback for sonar data
def sonar_callback(msg, state):
    state['sonar_data'] = (msg.point.x, msg.point.y)


# Callback for odometry data
def odom_callback(msg, state):
    state['odom_data'] = msg.pose.pose


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


def main():
    # Initialize node
    rospy.init_node("algo")

    # Create state dictionary
    state = {}

    # Subscribe to sonar and odometry topics
    rospy.Subscriber("/sonar_data", PointStamped, sonar_callback, state)
    rospy.Subscriber("/odom", Odometry, odom_callback, state)

    # Setup publisher for velocity commands
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(30)

    # Main loop
    while not rospy.is_shutdown():
        twist = Twist()

        # Calculate angular velocity for orientation
        vel_ang = orientar(state)

        # Set angular velocity in twist message
        if vel_ang is not None:
            twist.angular.z = vel_ang * 4

        # Set linear velocity in twist message
        twist.linear.y = 2

        # Publish twist message
        pub.publish(twist)

        rate.sleep()


if __name__ == "__main__":
    main()
