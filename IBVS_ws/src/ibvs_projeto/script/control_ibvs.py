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
from tf.transformations import quaternion_from_euler
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
        #self.pub_move = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size = 15)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.ibvs_calc)
        #rospy.Subscriber("/tag_detections_image", Image, self.image_proc)
    
    def ibvs_calc(self, msg):
        rho = 10e-6
        f = 0.0004621379699707031
        pp=[320, 240]
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

        u_20, v_20 = self.uv_mapping(x_20_calc, y_20_calc, rho, f, pp)
        u_36, v_36 = self.uv_mapping(x_36_calc, y_36_calc, rho, f, pp)
        u_52, v_52 = self.uv_mapping(x_52_calc, y_52_calc, rho, f, pp)

        cam = mvtb.CentralCamera(f=f,
                                rho=rho,
                                imagesize=[640, 480],
                                pp=pp,
                                name='ur5_realsense')
        jacob_20 = self.jacobian(u_20, v_20, Z_20, f, rho)
        jacob_36 = self.jacobian(u_36, v_36, Z_36, f, rho)
        jacob_52 = self.jacobian(u_52, v_52, Z_52, f, rho)
        jacob = np.concatenate((jacob_20, jacob_36, jacob_52), axis=0)
        jacob_inv = np.linalg.inv(jacob)

        lambda_ = 0.5
        dt = 0.1

        u1_goal, v1_goal = self.uv_mapping(-0.0966414748963234, 0.17414187520082974, rho, f, pp)
        u2_goal, v2_goal = self.uv_mapping(0.21511589930907404, 0.17841279418259656, rho, f, pp)
        u3_goal, v3_goal = self.uv_mapping(-0.09211395953208155, -0.13859814091486256, rho, f, pp)

        goal = np.array([u1_goal - u_20,
                        v1_goal - v_20,
                        u2_goal - u_36,
                        v2_goal - v_36,
                        u3_goal - u_52,
                        v3_goal - v_52], dtype=np.float64)

        v = lambda_ * np.matmul(jacob_inv, goal)

        movement = v*dt

        self.robot_planning(movement[0], movement[1], movement[2],
                            movement[3], movement[4], movement[5])

    def jacobian(self, u, v, z, f, rho):
        f_line = f/rho
        return np.array(
                [[-f_line/z, 0, u/z, u*v/f_line, -((f_line**2) + (u**2))/f_line, v],
                [0,   -f_line/z,   v/z, ((f_line**2) + (u**2))/f_line, -u*v/f_line, -u]]
                )

    def uv_mapping(self, x, y, rho, f, pp):
        u = (f/rho)*x + pp[0]
        v = (f/rho)*y + pp[1]
        return u, v

    def robot_planning(self, x_pose, y_pose, z_pose, roll, pitch, yaw):
        pose_goal = Pose()
        w_orient, x_orient, y_orient, z_orient = self.get_orientation(roll, pitch, yaw)
        pose_goal.orientation.w = w_orient
        pose_goal.orientation.x = x_orient
        pose_goal.orientation.y = y_orient
        pose_goal.orientation.z = z_orient
        pose_goal.position.x = x_pose
        pose_goal.position.y = y_pose
        pose_goal.position.z = z_pose

        self.move_group.set_pose_target(pose_goal)
        for i in range(5):
            self.move_group.plan()
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()   
  
    def get_orientation(self, roll, pitch, yaw):
        (w, x, y, z) = quaternion_from_euler (roll, pitch, yaw, 'rxyz')
        return w, x, y, z

controller = IBVS_Controller();

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()