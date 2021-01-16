#!/usr/bin/env python2
import rospy
from std_srvs.srv import SetBool
import os 

def control(request):
    os.system("shutdown now")

if __name__ == "__main__":
    rospy.init_node('shooterDown')
    service = rospy.Service('shootDown', SetBool, control)
    while not rospy.is_shutdown():
        rospy.spin()
    