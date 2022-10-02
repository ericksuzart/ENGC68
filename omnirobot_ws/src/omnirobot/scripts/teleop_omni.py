#!/usr/bin/env python3

from cmath import sqrt
from std_msgs.msg import Float64
import sys, select, termios, tty
import rospy

msg = """
Reading from the keyboard  
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 1 unit

anything else : stop

CTRL-C to quit
"""
a = sqrt(3).real/3
# v_0, v_1, v_2 are the velocities of the three wheels
moveBindings = {
  'i':(-a, 0,a),
  'o':(1,-1,0),
  'j':(-1/3,2/3,-1/3),
  'l':(1/3,-2/3,1/3),
  'u':(-1,0,1),
  ',':(a, 0,-a),
  '.':(1,0,-1),
  'm':(-1,1,0),  
}

speedBindings = {
  'q':(1),
  'z':(-1),
}

def getKey():
  tty.setraw(sys.stdin.fileno())
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)

  rospy.init_node('vel_Publisher')
  pub = rospy.Publisher('/open_base/back_joint_velocity_controller/command', Float64, queue_size=1)
  pub1 = rospy.Publisher('/open_base/left_joint_velocity_controller/command', Float64, queue_size=1)
  pub2 = rospy.Publisher('/open_base/right_joint_velocity_controller/command', Float64, queue_size=1)

  x = 0
  y = 0
  z = 0
  speed = float(10.0)
  print(msg)

  while(1):  
    key = getKey()   

    if key in moveBindings.keys():
      x = float(moveBindings[key][0])
      y = float(moveBindings[key][1])
      z = float(moveBindings[key][2])

    elif key in speedBindings.keys():
      speed += float(speedBindings[key])

    else:
      x = 0
      y = 0
      z = 0

    if (key == '\x03'):
      break  

    vel = float(x * speed)
    vel1 = float(y * speed)
    vel2 = float(z * speed)

    pub.publish(vel)
    pub1.publish(vel1)
    pub2.publish(vel2)
