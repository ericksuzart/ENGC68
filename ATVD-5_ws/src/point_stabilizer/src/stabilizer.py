#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg  import Twist
from tf.transformations import euler_from_quaternion
from math import cos, sin, sqrt, atan, pi
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates

x_goal = float(sys.argv[1])
y_goal = float(sys.argv[2])

class Stabilizer(object):
    def __init__(self):
        self.pub_move = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size = 15)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.point_stabilizer_move)
    
    def point_stabilizer_move(self, msg):
        x_gt, y_gt, yaw_gt = self.get_ground_truth()
        dx = x_goal - x_gt
        dy = y_goal - y_gt
        ro = sqrt((dx**2) + (dy**2))
        alpha = atan(dy/dx) - yaw_gt
        beta = -atan(dy/dx)
        kp = 1.2
        ka = 4.5
        kb = -0.6
        velocity = kp * ro
        vx = velocity * cos(yaw_gt) * 100 * abs(dx)
        vy = velocity * sin(yaw_gt) * 100 * abs(dy)
        gamma = (ka * alpha) + (kb * beta)
        movement = Twist()
        movement.linear.x = vx
        if dx * vx < 0: movement.linear.x *= -1
        movement.linear.y = vy
        if dy * vy < 0: movement.linear.y *= -1
        movement.linear.z = 0
        movement.angular.x = 0
        movement.angular.y = 0
        movement.angular.z = gamma
        if(abs(dy) < 0.1 and abs(dx) < 0.1):
            movement.linear.x = 0
            movement.linear.y = 0
            movement.angular.z = 0
            rospy.loginfo("Chegou à posição")

        self.pub_move.publish(movement)

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

rospy.init_node('stabilizer', anonymous=True)

stabil_instance = Stabilizer();

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()