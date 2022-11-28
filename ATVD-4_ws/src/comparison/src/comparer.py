#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg  import Twist, Point, Pose2D
from sensor_msgs.msg  import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos,sin
from gazebo_msgs.srv import GetModelState

class Comparer(object):
    def __init__(self):
        self.pub1 = rospy.Publisher('/diff_filtered_odometry', Pose2D, queue_size = 15)
        self.pub2 = rospy.Publisher('/diff_odometry', Pose2D, queue_size = 15)
        self.pub_gt = rospy.Publisher('/gt', Point, queue_size=10)
        self.comparer()

    def update_filtered_odometry(self, msg):
        now = rospy.get_rostime()
        gt = self.get_ground_truth()
        yaw = self.get_rotation(msg.pose.pose.orientation)
        diff_x = msg.pose.pose.position.x - gt[0]
        diff_y = msg.pose.pose.position.y - gt[1]
        diff_yaw = yaw - gt[2]
        self.pub1.publish(Pose2D(diff_x, diff_y, diff_yaw))
        log_str = "Diff_Filtered_X: " + str(diff_x)
        log_str = log_str + "\nDiff_Filtered_Y: " + str(diff_y)
        log_str = log_str + "\nDiff_Filtered_Yaw: " + str(diff_yaw)
        rospy.loginfo(log_str)
        return

    def update_odometry(self, msg):
        now = rospy.get_rostime()
        gt = self.get_ground_truth()
        yaw = self.get_rotation(msg.pose.pose.orientation)
        diff_x = msg.pose.pose.position.x - gt[0]
        diff_y = msg.pose.pose.position.y - gt[1]
        diff_yaw = yaw - gt[2]
        self.pub2.publish(Pose2D(diff_x, diff_y, diff_yaw))
        log_str = "Diff_X: " + str(diff_x)
        log_str = log_str + "\nDiff_Y: " + str(diff_y)
        log_str = log_str + "\nDiff_Yaw: " + str(diff_yaw)
        rospy.loginfo(log_str)
        return

    def get_ground_truth(self):
        now = rospy.get_rostime()
        g_get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            state = g_get_state(model_name="husky")
        except Exception(e):
            rospy.logerr('Error on calling service: %s',str(e))
            return
        x_gt = state.pose.position.x
        y_gt = state.pose.position.y 
        yaw_gt = self.get_rotation(state.pose.orientation)
        #self.outbag.write('/gt', Pose2D(x_gt, y_gt, yaw_gt), now)
        return [x_gt, y_gt, yaw_gt]

    def comparer(self):
        rospy.Subscriber("/odometry/filtered", Odometry, self.update_filtered_odometry)
        rospy.Subscriber("/husky_velocity_controller/odom", Odometry, self.update_odometry)
        rospy.loginfo("Teste")
        return
    
    def get_rotation(self, msg):
        orientation_q = msg
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

rospy.init_node('comparer', anonymous=True)

el = Comparer();

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()