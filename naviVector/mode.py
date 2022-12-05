import numpy as np
import math

class Mode():
    def __init__(self, path, EM_weight, goal_pose, switch_range):
        self.path = path
        self.command = np.zeros(2)
        self.real_goal_pose = [7, -1]
        self.switch_range = switch_range #switch is a range
        self.next_topo_point = 0
        self.EM = EM_weight
        self.goal_pose = goal_pose
        self.subgoals_num = 0
        
        self.goal_vector = np.zeros(2)
        self.wall_vector = np.zeros(2)
        self.final_vector = np.zeros(2)
        self.zero_range = 0.05
        
        self.distance_to_walls = 0
        self.head_distance_to_wall = 0
        
    def compute_vector(self, current_pose, scans):
        self.distance_to_wall(scans)
        print(min(scans))
        if (min(scans) >= self.switch_range):
            print('reset subgoal')
            self.goal_pose = [7, -1]
            print(self.goal_pose) 
        else:
            print('set subgoal')
            if self.head_distance_to_wall < self.switch_range:
                angle = current_pose[2] + self.distance_to_walls*math.pi/500 - math.pi
                vector = [math.cos(angle), math.sin(angle)]            
                tempsubgoal = np.zeros(2)
                subgoals = []
                for point in self.path:
                    tempsubgoal[0] = (self.EM[point, 0] - current_pose[0])/(
                        math.pow(self.EM[point, 0] - current_pose[0], 2)+
                        math.pow(self.EM[point, 1] - current_pose[1], 2))
                    tempsubgoal[1] = (self.EM[point, 1] - current_pose[1])/(
                        math.pow(self.EM[point, 0] - current_pose[0], 2)+
                        math.pow(self.EM[point, 1] - current_pose[1], 2))
                    
                    vectors_dot = abs(np.dot(vector, tempsubgoal))
                    subgoals.append(vectors_dot)
                    self.subgoals_num = np.argmin(subgoals)
                self.goal_pose[0] = self.EM[self.subgoals_num, 0]
                self.goal_pose[1] = self.EM[self.subgoals_num, 1]
        
        check_range_code = self.check_goal(current_pose)
        
        if check_range_code==0:
            print('get goal')
        elif check_range_code==1:
            self.subgoals_num += 1
        
        self.goal_vector[0] = (self.goal_pose[0] - current_pose[0])/math.sqrt(
            math.pow(self.goal_pose[0] - current_pose[0], 2)+
            math.pow(self.goal_pose[1] - current_pose[1], 2))
        self.goal_vector[1] = (self.goal_pose[1] - current_pose[1])/math.sqrt(
            math.pow(self.goal_pose[0] - current_pose[0], 2)+
            math.pow(self.goal_pose[1] - current_pose[1], 2))
        
        #follow wall
        ranges = 0.8
        if scans[249]<ranges:
            self.wall_vector[1] = (0.5-scans[249])/abs(ranges-scans[249])
            self.wall_vector[0] = 0
        else:
            self.wall_vector = np.zeros(2)
        if scans[749]<ranges:
            self.wall_vector[1] = (scans[749] - ranges)/abs(scans[749] - 0.5)
            self.wall_vector[0] = 0
        else:
            self.wall_vector = np.zeros(2)
        #get final vector
        self.final_vector = self.goal_vector + self.wall_vector
        print(self.goal_pose, self.final_vector, self.wall_vector, self.goal_vector)
        return self.final_vector
    
    def distance_to_wall(self, scans):
        self.head_distance_to_wall = scans[499]
        self.distance_to_walls = np.argmin(scans)
        print(self.head_distance_to_wall, scans[self.distance_to_walls])
        
    def check_goal(self, current_pose):
        to_subgoal_range = math.sqrt(math.pow(self.goal_pose[0]-current_pose[0], 2)+
                                     math.pow(self.goal_pose[1]-current_pose[1], 2))
        to_goal_range = math.sqrt(math.pow(7-current_pose[0], 2)+
                                     math.pow(-1-current_pose[1], 2))
        if to_goal_range<self.zero_range:
            return 0
        elif to_subgoal_range<self.zero_range:
            return 1
        else:
            return 2
        
    

    
                
        
