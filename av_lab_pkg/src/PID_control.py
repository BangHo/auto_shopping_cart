#!/usr/bin/env python3
#조향각 제어하는 PID 알고리즘

import rospy
import math
import time
from geometry_msgs.msg import Twist,Point

LINEAR_VEL_MAX = 0.20           #속도 최댓값
LINEAR_VEL_MIN = 0.1            #속도 최솟값
STOP_DISTANCE = 0.4





class PID():
  def __init__(self,kp,ki,kd):
    self.kp = kp                  #P_gain 초기화
    self.ki = ki                  #I_gain 초기화
    self.kd = kd                  #D_gain 초기화
    self.p_error = 0.0            #P_error 초기화
    self.i_error = 0.0            #I_error 초기화
    self.d_error = 0.0            #D_error 초기화
    
  def pid_control(self, cte):
    self.d_error = cte-self.p_error       #D_error 계산
    self.p_error = cte                    #P_error 계산
    self.i_error += cte                   #I_error 계산
    return self.kp*self.p_error + self.ki*self.i_error + self.kd*self.d_error     #PD 계산값 리턴


class Moving():
  def __init__(self):
    self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.sub = rospy.Subscriber()
    self.moving()
    self.angle=0.0

  def get_odom(self, odom):
    self.position = Point()
    self.orientation = odom.pose.pose.orientation

  def moving(self):
    twist = Twist()
    rospy.loginfo('start')





def main():
  rospy.init_node('PID_control')
