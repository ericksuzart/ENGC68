#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg  import Twist, Point
from sensor_msgs.msg  import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos,sin
from gazebo_msgs.srv import GetModelState

def talker():
    topic = 'chatter'
    pub = rospy.Publisher(topic, String)
    pub_gt = rospy.Publisher('/gt', Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.loginfo("I will publish to the topic %s", topic)
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
        yaw_gt = state.pose.orientation
        theta_gt = yaw_gt 
        pub_gt.publish(Point(x_gt, y_gt, y_gt))
 
if __name__ == '__main__':
    talker()