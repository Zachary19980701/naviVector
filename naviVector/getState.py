import math
from operator import le
from select import select
from threading import local
from time import sleep
from turtle import pos, position
from sensor_msgs.msg import LaserScan
import rospy
import numpy as np
import networkx as nx
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy import Rate
import time
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import *
import matplotlib.pyplot as plt


class GetState():
    def __init__(self):
        self.state = np.zeros(3)
        self.laser_state = np.zeros(4)
    
    
    def get_model_state(self):   
        rospy.wait_for_message('/gazebo/model_states', ModelStates)
        get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model = GetModelStateRequest()
        model.model_name = 'turtlebot3_burger'
        objstate = get_state_service(model)
        self.state[0] = objstate.pose.position.x
        self.state[1] = objstate.pose.position.y
        _, _, angle = self.quaternion_to_euler(objstate.pose.orientation.x, objstate.pose.orientation.y, 
                                               objstate.pose.orientation.z, objstate.pose.orientation.w)
        self.state[2] = angle

 
    #quaternion_to_euler
    def quaternion_to_euler(self, x, y , z, w):
        pitch = math.atan2(2 * (w*z + x*y), 1 - 2*(z*z + y*y))
        heading = 0
        roll = 0
        return heading, roll, pitch
    
    def get_barrier_state(self):
        scan = rospy.wait_for_message('/scan', LaserScan)
        scans = list(scan.ranges)
        for i in range(len(scans)):
            if scans[i] == np.inf:
                scans[i] = 3.5
                
            if scans[i] > 1.0:
                scans[i] = 1.0

        self.laser_state = np.asarray(scans)
    
    def get_state(self):
        self.get_barrier_state()
        self.get_model_state()
        
        return self.state, self.laser_state