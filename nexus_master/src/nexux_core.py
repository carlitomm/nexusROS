#!/usr/bin/env python

import rospy
import time
import actionlib

from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import dynamic_reconfigure.client

class nexus_core:

    def __init__(self):
        
        """
        Publishers
        """
        self.state_publisher = rospy.Publisher('state_string', String)
        
        """
        Services
        """
        self.shootDown_client = rospy.ServiceProxy('shootDown', SetBool)

        """
        Dynamic recofgure
        """
        self.client_reconfigure = dynamic_reconfigure.client.Client("move_base", timeout=30, config_callback=self.dynamic_callback)
       
        #self.client_reconfigure.update_configuration({"base_local_planner":"base_local_planner/TrajectoryPlannerROS"})
        #self.client_reconfigure.update_configuration({"base_local_planner":"dwa_local_planner/DWAPlannerROS"})
        #self.client_reconfigure.update_configuration({"rtabmap/Grid/FromDepth":"false"})
        #self.client_reconfigure.update_configuration({"dwa":False}) 
    
    def dynamic_callback(self, config):
        print("configuration done :)")
    
if __name__ == "__main__":
    
    rospy.init_node('nexus_core')
    Nexus_core = nexus_core()
    rate = rospy.Rate(10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")