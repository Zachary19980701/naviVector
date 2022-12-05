import rospy
from geometry_msgs.msg import Twist
import math
import numpy as np
from rospy import Rate


class Drive():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.msg = Twist()
        self.rate = Rate(1)
        
    def trans_angle(self, current_pose, angle):
        '''
        just because the robot pose was define as -pi to pi, so if not trans angle,
        the robot would turn the large angle
        '''
        #compute the urge angle between now angle and target angle
        if current_pose<-math.pi/2 and angle>math.pi/2:
            urge_angle = -(abs(current_pose + math.pi) + abs(math.pi - angle))
        elif current_pose>math.pi/2 and angle<-math.pi/2:
            urge_angle = (abs(angle + math.pi) + abs(math.pi - current_pose))
        else:
            urge_angle = angle - current_pose
        return urge_angle

    def drive(self, current_pose, vector):
        target_angle = math.atan2(vector[1], vector[0])
        current_angle = current_pose[2]
        urge_angle = self.trans_angle(current_angle, target_angle)
                
        #drive car
        
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.x = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0
        
        if urge_angle > 0.05:
            self.msg.angular.z = urge_angle
            self.msg.linear.x = 0
        else:
            self.msg.angular.z = 0.5 * urge_angle
            self.msg.linear.x = 0.05

        #publish /cmd_vel
        self.pub.publish(self.msg)
        self.rate.sleep()