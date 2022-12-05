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



path = np.zeros(2)


def get_model_state():   
    rospy.wait_for_message('/gazebo/model_states', ModelStates)
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name = 'turtlebot3_burger'
    objstate = get_state_service(model)
    return objstate

def dataSave(odom):

        
    global path  
    positionX = odom.pose.position.x
    positionY = odom.pose.position.y
    tempPosition = np.zeros(2)
    tempPosition[0] = positionX
    tempPosition[1] = positionY
    print(tempPosition)
    path = np.vstack((path, tempPosition))
    np.savetxt("/home/yjs/Yproject/path2.txt", path)
    print("save path")
    
def listener():

    rospy.init_node('path' , anonymous=True)
    while(1):
        print('work')
        odom = get_model_state()
        dataSave(odom)
    rospy.spin()
    
def plot():
    path = np.loadtxt("/home/yjs/Yproject/path2.txt")
    plt.plot(path[:, 0], path[:, 1])
    plt.show()

if __name__=='__main__':
    plot()
