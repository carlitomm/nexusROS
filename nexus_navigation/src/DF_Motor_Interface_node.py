#!/usr/bin/env python

import time
import math as mt
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board
import rospy
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import tf


"""
ROS iNTERFACE
"""
class ros_interface():
  def __init__(self):
     
    """
    velovity parameters
    """
    self.velocity = Twist()
    self.speedMMPS = 0
    self.rad = 0
    self.omega = 0

    """
    Odometry Parameters
    """
    self.x = 0.0
    self.y = 0.0
    self.th = 0.0

    self.vx =  0.0
    self.vy =  0.0
    self.vth =  0.0

    self.odom_broadcaster = tf.TransformBroadcaster()

    """
    ros subscribers and publishers
    """ 
    self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_cb)
    self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
  
  """
  odomatry funcions
  """
  def compute_odometry(self, ULSpeedMMPS, LLSpeedMMPS, LRSpeedMMPS, URSpeedMMPS):
    w1 = URSpeedMMPS
    w2 = ULSpeedMMPS
    w3 = LLSpeedMMPS
    w4 = LRSpeedMMPS
    r = 20 #dummy wheel radius
    R = 20 #dummy radious of the Robot
    self.th = (r/2)*( w1*(1/2*R) + w2*(1/2*R) + w3*(1/2*R) + w4*(1/2*R) )
    self.vx = (r/2)*(-w1*(mt.sin(self.th + mt.pi/4)) - w2*(mt.sin(self.th + 3*mt.pi/4)) - w3*(mt.sin(self.th + 5*mt.pi/4)) - w4*(mt.sin(self.th + 7*mt.pi/4)))
    self.vy = (r/2)*( w1*(mt.cos(self.th + mt.pi/4)) + w2*(mt.cos(self.th + 3*mt.pi/4)) + w3*(mt.cos(self.th + 5*mt.pi/4)) + w4*(mt.cos(self.th + 7*mt.pi/4)))
    

  def publish_odom(self):   
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
    current_time = rospy.Time.now()
    self.odom_broadcaster.sendTransform(
       (self.x, self.y, 0.),
       odom_quat,
       current_time,
       "base_link",
       "odom"
    )
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

    self.odom_pub.publish(odom)

  """
  velocity functions
  """ 
  def VelCart2VelEuler(self):
    self.speedMMPS = mt.hypot(self.velocity.linear.x, self.velocity.linear.y)
    self.rad = mt.atan2(self.velocity.linear.x, self.velocity.linear.y)
    self.omega = self.velocity.angular.z

  def cmd_cb(self, msg):
    self.velocity = msg
    self.VelCart2VelEuler()



"""
MOTOR iNTERFACE
"""
class motor_interface():
  def __init__(self):
    self.WHEELSPAN = 300
    self.upperboard = Board(1, 0x10)    # Select bus 1, set address to 0x10
    self.lowerboard = Board(1, 0x20)

    while self.upperboard.begin() != self.upperboard.STA_OK:    # Board begin and check board status
      self.print_board_status()
      print("board begin faild")
      time.sleep(2)
    while self.lowerboard.begin() != self.lowerboard.STA_OK:    # Board begin and check board status
      self.print_board_status()
      print("board begin faild")
      time.sleep(2)
    
    print("board begin success")

    self.upperboard.set_encoder_enable(self.upperboard.ALL)                 
    self.upperboard.set_encoder_reduction_ratio(self.upperboard.ALL, 43)

    self.lowerboard.set_encoder_enable(self.lowerboard.ALL)                 
    self.lowerboard.set_encoder_reduction_ratio(self.lowerboard.ALL, 43)

    self.upperboard.set_moter_pwm_frequency(1000)
    self.lowerboard.set_moter_pwm_frequency(1000)
  
  def setCarMove(self, speedMMPS, rad, omega):
    self.wheelULSetSpeedMMPS(speedMMPS*mt.sin(rad)+speedMMPS*mt.cos(rad)-omega*self.WHEELSPAN)
    self.wheelLLSetSpeedMMPS(speedMMPS*mt.sin(rad)-speedMMPS*mt.cos(rad)-omega*self.WHEELSPAN)
    self.wheelLRSetSpeedMMPS(-(speedMMPS*mt.sin(rad)+speedMMPS*mt.cos(rad)+omega*self.WHEELSPAN))
    self.wheelURSetSpeedMMPS(-(speedMMPS*mt.sin(rad)-speedMMPS*mt.cos(rad)+omega*self.WHEELSPAN))

  """
  set and get speed of every wheel
  """
  def wheelULSetSpeedMMPS(self, speedMMPS):
    self.upperboard.motor_movement([self.upperboard.M1], self.upperboard.CW, speedMMPS)    

  def wheelULGetSpeedMMPS(self):
    speed = self.upperboard.get_encoder_speed(self.upperboard.ALL)
    return speed[0]

  def wheelLLSetSpeedMMPS(self, speedMMPS):
    self.lowerboard.motor_movement([self.lowerboard.M1], self.lowerboard.CW, speedMMPS)

  def wheelLLGetSpeedMMPS(self):
    speed = self.lowerboard.get_encoder_speed(self.lowerboard.ALL)
    return speed[0]   
     
  def wheelLRSetSpeedMMPS(self, speedMMPS):   
    self.lowerboard.motor_movement([self.lowerboard.M2], self.lowerboard.CCW, speedMMPS)

  def wheelLRGetSpeedMMPS(self):
    speed = self.lowerboard.get_encoder_speed(self.lowerboard.ALL)
    return speed[1] 
    
  def wheelURSetSpeedMMPS(self, speedMMPS):   
    self.upperboard.motor_movement([self.upperboard.M2], self.upperboard.CCW, speedMMPS)
  
  def wheelURGetSpeedMMPS(self):
    speed = self.upperboard.get_encoder_speed(self.upperboard.ALL)
    return speed[1]
 
  def board_detect(self):
    u = self.upperboard.detecte()
    l = self.lowerboard.detecte()
    print("Board list conform:")
    print(u,l)

  def print_board_status(self):
    if self.upperboard.last_operate_status == self.upperboard.STA_OK:
      print("board status: everything ok")
    elif self.upperboard.last_operate_status == self.upperboard.STA_ERR:
      print("board status: unexpected error")
    elif self.upperboard.last_operate_status == self.upperboard.STA_ERR_DEVICE_NOT_DETECTED:
      print("board status: device not detected")
    elif self.upperboard.last_operate_status == self.upperboard.STA_ERR_PARAMETER:
      print("board status: parameter error, last operate no effective")
    elif self.upperboard.last_operate_status == self.upperboard.STA_ERR_SOFT_VERSION:
      print("board status: unsupport board framware version")

  def setCarStop(self):
    self.upperboard.motor_stop(self.upperboard.ALL)
    self.lowerboard.motor_stop(self.lowerboard.ALL)
    
  def setBoard_addr(self, upperadd = 0x10, loweraddr = 0x20):
    # Set board controler address, use it carefully, reboot 
    #module to make it effective

    self.upperboard.set_addr(0x10)
    if self.upperboard.last_operate_status != self.upperboard.STA_OK:
      print("set board address faild")
    else:
      print("set board address success")

    self.lowerboard.set_addr(0x10)
    if self.lowerboard.last_operate_status != self.lowerboard.STA_OK:
      print("set board address faild")
    else:
      print("set board address success")

if __name__ == "__main__":
    rospy.init_node('DF_Motor_Interface')
    rate = rospy.Rate(10)

    Motor_interface = motor_interface()
    Ros_interface = ros_interface()
    
    while not rospy.is_shutdown():
      rate.sleep()

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")