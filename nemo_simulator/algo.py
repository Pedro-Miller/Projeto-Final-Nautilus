import rospy
import tf2_ros as tf

from geometry_msgs.msg import Twist, Pose, Vector3
import math





# Callback sonar
def sonar_callback(msg):
    x_m = msg.x
    y_m = msg.y
    return (x_m,y_m)

def odom_callback(msg):
    
    px = msg.position.x
    py = msg.position.y

    orientation = msg.orientation

    # Create a tf.Transform object with the Quaternion orientation as the rotation
    tf_transform = tf.TransformerROS().fromTranslationRotation([0, 0, 0], [orientation.x, orientation.y, orientation.z, orientation.w])
    # Use the tf.transformations.euler_from_quaternion function to convert the orientation to Euler angles
    angulos_euler = tf.transformations.euler_from_quaternion(tf_transform.getRotation())

    # Extract the yaw angle from the Euler angles
    yaw = angulos_euler[2]
    return (px,py,yaw)


def orientar():
    # calcula diferença de angulo entre carros
    odom = odom_callback()
    sonar = sonar_callback()

    dx = sonar[0] - odom[0]
    dy = sonar[1] - odom[1]

    target_angle = math.atan2(dy, dx)
    current_angle =  odom[2]

    angle_erro = target_angle - current_angle

    # normaliza a diferença para -pi e pi
    while angle_erro > math.pi:
        angle_erro -= 2 * math.pi
    while angle_erro < -math.pi:
        angle_erro += 2 * math.pi

    # retorna velocidade angular necessaria para orientação
    velang = Vector3(z=angle_erro)
    return velang  
    

if __name__ == "__main__":

    rospy.init_node("algo")

    rospy.Subscriber("/odom", Pose, odom_callback)


    rospy.Subscriber("/sonar_data", Twist, sonar_callback)


    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(10)

    # Main loop
    while not rospy.is_shutdown():

        twist = Twist()

        twist.angular = orientar()
        twist.linear.y = 50

        pub.publish(twist)

        rate.sleep()




    



