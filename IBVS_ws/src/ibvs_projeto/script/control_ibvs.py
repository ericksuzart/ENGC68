#!/usr/bin/env python
import rospy
import sys
import copy
import numpy as np
import machinevisiontoolbox as mvtb
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
from math import cos, sin, sqrt, atan, pi
from apriltag_ros.msg import AprilTagDetectionArray

class IBVS_Controller(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ibvs_controller', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.robot_planning(0.1, 0.1, 0.2, 0.1, 0, 0, 0)
        #self.pub_move = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size = 15)
        #rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.ibvs_calc)
        #rospy.Subscriber("/tag_detections_image", Image, self.image_proc)
    
    def ibvs_calc(self, msg):
        X_20 = msg.detections[0].pose.pose.pose.position.x
        Y_20 = msg.detections[0].pose.pose.pose.position.y
        Z_20 = msg.detections[0].pose.pose.pose.position.z
        X_36 = msg.detections[1].pose.pose.pose.position.x
        Y_36 = msg.detections[1].pose.pose.pose.position.y
        Z_36 = msg.detections[1].pose.pose.pose.position.z
        X_52 = msg.detections[2].pose.pose.pose.position.x
        Y_52 = msg.detections[2].pose.pose.pose.position.y
        Z_52 = msg.detections[2].pose.pose.pose.position.z

        x_20_calc = X_20/Z_20
        y_20_calc = Y_20/Z_20
        x_36_calc = X_36/Z_36
        y_36_calc = Y_36/Z_36
        x_52_calc = X_52/Z_52
        y_52_calc = Y_52/Z_52

        cam = mvtb.CentralCamera(f=0.0004621379699707031,
                                rho=10e-6,
                                imagesize=[640, 480],
                                pp=[320, 240],
                                name='ur5_realsense')
        jacob_20 = np.array(cam.visjac_p((X_20, Y_20), Z_20))
        jacob_36 = np.array(cam.visjac_p((X_36, Y_36), Z_36))
        jacob_52 = np.array(cam.visjac_p((X_52, Y_52), Z_52))
        jacob = np.concatenate((jacob_20, jacob_36, jacob_52), axis=0)
        jacob_inv = np.linalg.inv(jacob)

        lambda_ = 0.7

        goal = np.array([-0.0966414748963234 - x_20_calc,
                        0.17414187520082974 - y_20_calc,
                        0.21511589930907404 - x_36_calc,
                        0.17841279418259656 - y_36_calc,
                        -0.09211395953208155 - x_52_calc,
                        -0.13859814091486256 - y_52_calc])

        v = lambda_ * np.matmul(jacob_inv, goal)
        print(v)
        return

    def robot_planning(self, x_pose, y_pose, z_pose, w_orient, x_orient, y_orient, z_orient):
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0
        #pose_goal.orientation.x = x_orient
        #pose_goal.orientation.y = y_orient
        #pose_goal.orientation.z = z_orient
        pose_goal.position.x = x_pose
        pose_goal.position.y = y_pose
        pose_goal.position.z = z_pose

        self.move_group.set_pose_target(pose_goal)
        #success = self.move_group.plan()
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()
                

    '''   
    def get_rotation(self, msg):
        orientation_q = msg
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw
    '''

controller = IBVS_Controller();

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()