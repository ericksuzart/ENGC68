#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg  import Twist, Point
from sensor_msgs.msg  import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos,sin
from gazebo_msgs.srv import GetModelState

def comparer():
    pub = rospy.Publisher('chatter', String)
    pub_gt = rospy.Publisher('/gt', Point, queue_size=10)
    rospy.init_node('comparer', anonymous=True)
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        rospy.sleep(0.1)
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
 
def get_rotation (msg):
    orientation_q = msg
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw

if __name__ == '__main__':
    comparer()