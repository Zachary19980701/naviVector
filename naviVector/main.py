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

from getState import GetState
from PathCompute import PathCompute
from drive import Drive
from mode import Mode

def main():
    rospy.init_node('laserSubscriber', anonymous=True)
    map_path = '/home/yjs/Yproject/'
    get_state = GetState()
    current_pose, scan_state = get_state.get_state()
    path_compute = PathCompute(map_path)
    path = path_compute.topo_path_compute(current_pose, [7, -1])
    drive = Drive()
    EM_weight = np.loadtxt(map_path + 'EM_weight.txt')
    mode1 = Mode(path, EM_weight, [7, -1], 0.4)
    
    while(1):
        current_pose, scan_state = get_state.get_state()
        target_vector = mode1.compute_vector(current_pose, scan_state)
        drive.drive(current_pose, target_vector)
        
if __name__=='__main__':
    main()