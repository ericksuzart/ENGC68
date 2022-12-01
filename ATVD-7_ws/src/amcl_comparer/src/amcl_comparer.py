#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState

class AMCL_Comparer(object):
    def __init__(self):
        self.pub_diff = rospy.Publisher('/amcl_gt_diff', Twist, queue_size = 15)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.compare)
    
    def compare(self, msg):
        x_gt, y_gt, yaw_gt = self.get_ground_truth()
        x_amcl = msg.pose.pose.position.x
        y_amcl = msg.pose.pose.position.y
        yaw_amcl = self.get_rotation(msg.pose.pose.orientation)
        diff_x = x_gt - x_amcl
        diff_y = y_gt - y_amcl
        diff_yaw = yaw_gt - yaw_amcl
        self.pub_diff.publish(Pose2D(diff_x, diff_y, diff_yaw))
        return

    def get_ground_truth(self):
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
        return (x_gt, y_gt, yaw_gt)
    
    def get_rotation(self, msg):
        orientation_q = msg
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

rospy.init_node('amcl_comparer', anonymous=True)

amcl_comp_instance = AMCL_Comparer();

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
