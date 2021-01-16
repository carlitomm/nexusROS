#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math as mt
from nav_msgs.msg import Odometry

class wheel_velocity_publisher():
    def __init__(self):
        self.wheel_pub1 = rospy.Publisher('wheel1', Float32, queue_size=0)
        self.wheel_pub2 = rospy.Publisher('wheel2', Float32, queue_size=0)
        self.wheel_pub3 = rospy.Publisher('wheel3', Float32, queue_size=0)
        self.wheel_pub4 = rospy.Publisher('wheel4', Float32, queue_size=0)

        self.velx_pub  = rospy.Publisher('vx', Float32, queue_size=0)
        self.vely_pub  = rospy.Publisher('vy', Float32, queue_size=0)
        self.velth_pub = rospy.Publisher('vth', Float32, queue_size=0)

        self.cmdvel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmdvel_cb)
        self.odomet_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)

        self.wheel1 = Float32()
        self.wheel2 = Float32()
        self.wheel3 = Float32()
        self.wheel4 = Float32()

        self.Robot_Radius = 0.35
        self.wheel_radius = 0.25

        self.th = 0
    
    def odom_cb(self, msg):
        self.th = msg.pose.pose.orientation.z

    def cmdvel_cb(self, msg):
        R = self.Robot_Radius
        r = self.wheel_radius
        th = self.th
        pi = mt.pi

        vx = msg.linear.x
        vy = msg.linear.y
        vth = msg.angular.z

        self.wheel1 = 1/r*(-mt.sin(th + 1*pi/4)*vx + mt.cos(th + 1*pi/4)*vy + R*vth)
        self.wheel2 = 1/r*(-mt.sin(th + 3*pi/4)*vx + mt.cos(th + 3*pi/4)*vy + R*vth)
        self.wheel3 = 1/r*(-mt.sin(th + 5*pi/4)*vx + mt.cos(th + 4*pi/4)*vy + R*vth)
        self.wheel4 = 1/r*(-mt.sin(th + 7*pi/4)*vx + mt.cos(th + 5*pi/4)*vy + R*vth)

        self.wheel_pub1.publish(self.wheel1)
        self.wheel_pub2.publish(self.wheel2)
        self.wheel_pub3.publish(self.wheel3)
        self.wheel_pub4.publish(self.wheel4)

        self.velx_pub.publish(vx)
        self.vely_pub.publish(vy)
        self.velth_pub.publish(vth)
    
if __name__ == "__main__":
    rospy.init_node('wheel_velocity_visualization')
    wheels = wheel_velocity_publisher()
    while not rospy.is_shutdown():
        rospy.spin()
