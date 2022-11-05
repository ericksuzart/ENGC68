#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg  import Twist, Point
from sensor_msgs.msg  import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos,sin
from gazebo_msgs.srv import GetModelState


def update_filtered_odometry(msg):
    str = "hello world %s"%msg.pose.pose.position.x
    rospy.loginfo(str)
    rospy.sleep(0.1)
    return

def update_odometry(msg):
    str = "hello world2 %s"%msg.pose.pose.position.x
    rospy.loginfo(str)
    rospy.sleep(0.1)
    return

def comparer():
    pub_gt = rospy.Publisher('/gt', Point, queue_size=10)
    rospy.init_node('comparer', anonymous=True)
    rospy.Subscriber("/odometry/filtered", Odometry, update_filtered_odometry)
    rospy.Subscriber("/husky_velocity_controller/odom", Odometry, update_odometry)
    while not rospy.is_shutdown():
        g_get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            state = g_get_state(model_name="husky")
        except Exception(e):
            rospy.logerr('Error on calling service: %s',str(e))
            return
        x_gt = state.pose.position.x
        y_gt = state.pose.position.y 
        yaw_gt = get_rotation(state.pose.orientation)
        theta_gt = yaw_gt 
        pub_gt.publish(Point(x_gt, y_gt, yaw_gt))
        str = "hello world3 %s"%x_gt
        rospy.loginfo(str)
        rospy.sleep(0.1)
 
def get_rotation (msg):
    orientation_q = msg
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw

if __name__ == '__main__':
    comparer()