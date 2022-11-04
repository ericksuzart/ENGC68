#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg  import Twist, Point
from sensor_msgs.msg  import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos,sin
from gazebo_msgs.srv import GetModelState


class ExLoc(object):

  x_gt = 0;
  y_gt = 0;
  theta_gt = 0;


  def __init__(self):

    rospy.Subscriber("/cmd_vel", Twist, self.odo_cb) 
    self.pub_gt = rospy.Publisher('/gt', Point, queue_size=10)






  def get_gt(self):
  
    g_get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    
    rospy.wait_for_service("/gazebo/get_model_state")
    
    try:
            
      state = g_get_state(model_name="husky")
            
    except Exception(e):
        
      rospy.logerr('Error on calling service: %s',str(e))
      return
    self.x_gt = state.pose.position.x
    self.y_gt = state.pose.position.y 
    yaw_gt = self.get_rotation(state.pose.orientation)
    self.theta_gt = yaw_gt 
    self.pub_gt.publish(Point(self.x_gt, self.y_gt, self.theta_gt))
    #print(self.theta_gt)    

 
    


  def odo_cb(self, msg):
    
    self.get_gt()


    
 
 

rospy.init_node('odo_vs_dr', anonymous=True)


el = ExLoc();


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
