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
        dx, dy, ro, alpha, beta = self.calc_var_change(x_gt, y_gt, yaw_gt)
        vx, vy, gamma = self.calc_twist_values(ro, alpha, beta, yaw_gt)
        self.move(vx, vy, gamma, dx, dy, yaw_gt)

    def calc_var_change(self, x_gt, y_gt, yaw_gt):
        dx = x_goal - x_gt
        dy = y_goal - y_gt
        theta = atan(dy/dx)
        yaw_goal = theta - yaw_gt
        ro = sqrt((dx**2) + (dy**2))

        alpha = yaw_goal
        if(alpha > pi/2):
            alpha -= pi
        elif(alpha < -pi/2):
            alpha += pi

        beta = -atan(dy/dx)

        return (dx, dy, ro, alpha, beta)

    def calc_twist_values(self, ro, alpha, beta, yaw):
        kp = 0.5
        ka = 1
        kb = 1
        velocity = kp * ro
        vx = velocity * cos(alpha)
        vy = velocity * sin(alpha)
        #gamma = (ka * alpha) - (kb * beta)
        gamma = 10 * alpha
        rospy.loginfo(str(alpha) + " " + str(beta))
        return (vx, vy, gamma)

    def move(self, vx, vy, gamma, dx, dy, yaw):
        movement = Twist()
        movement.linear.x = vx
        if dx * cos(yaw) < 0: movement.linear.x *= -1
        movement.linear.y = vy
        if dy * sin(yaw) > 0: movement.linear.y *= -1
        movement.linear.z = 0
        movement.angular.x = 0
        movement.angular.y = 0
        movement.angular.z = gamma
        self.pub_move.publish(movement)
        log_str = "vx = " + str(vx) + " vy = " + str(vy) + " th = " + str(yaw) + " gamma = " + str(gamma)
        rospy.loginfo(log_str)
        if(abs(dy) < 0.2 and abs(dx) < 0.2):
            movement.linear.x = 0
            movement.linear.y = 0
            movement.angular.z = 0
            self.pub_move.publish(movement)
            rospy.loginfo("Chegou à posição")
            rospy.signal_shutdown("Chegou à posição")
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

rospy.init_node('stabilizer', anonymous=True)

stabil_instance = Stabilizer();

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()