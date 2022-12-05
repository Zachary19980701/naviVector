import numpy as np
import math

class Mode():
    def __init__(self, path, EM_weight, goal_pose, switch_range):
        self.path = path
        self.command = np.zeros(2)
        self.goal_pose = goal_pose
        self.switch_range = switch_range #switch is a range
        self.next_topo_point = 0
        self.EM = EM_weight
        self.goal_vector = np.zeros(2)
        self.barrier_vector = np.zeros(2)
        self.subgoal_vector = np.zeros(2)
        self.final_vector = np.zeros(2)
        self.zero_range = 0.01
        
    def compute_vector(self, current_pose, scans):
        self.goal_vector[0] = (self.goal_pose[0]-current_pose[0])/(
                            math.pow(self.goal_pose[0]-current_pose[0], 2)+
                            math.pow(self.goal_pose[1]-current_pose[1], 2))
        self.goal_vector[1] = (self.goal_pose[1]-current_pose[1])/(
                            math.pow(self.goal_pose[0]-current_pose[0], 2)+
                            math.pow(self.goal_pose[1]-current_pose[1], 2))
        
        for i in range(len(scans)):
            self.barrier_vector[0] = -(1.0 - scans[i]) * math.cos((i*2*math.pi-math.pi)/len(scans))
            self.barrier_vector[1] = -(1.0 - scans[i]) * math.sin((i*2*math.pi-math.pi)/len(scans))

        self.subgoal_vector = np.zeros(2)
        
        if(np.dot(self.goal_vector, self.barrier_vector)<-0.7):
            #compute subgoal
            tempsubgoal = np.zeros(2)
            subgoals = []
            for point in reversed(self.path):
                tempsubgoal[0] = (self.EM[point, 0] - current_pose[0])/(
                    math.pow(self.EM[point, 0] - current_pose[0], 2)+
                    math.pow(self.EM[point, 1] - current_pose[1], 2))
                tempsubgoal[1] = self.EM[point, 1] - current_pose[1]/(
                    math.pow(self.EM[point, 0] - current_pose[0], 2)+
                    math.pow(self.EM[point, 1] - current_pose[1], 2))
                
                vectors_dot = abs(np.dot(self.goal_vector, tempsubgoal))
                subgoals.append(vectors_dot)
            
            self.subgoal_vector[0] = self.EM[np.argmin(subgoals), 0] - current_pose[0]
            self.subgoal_vector[1] = self.EM[np.argmin(subgoals), 1] - current_pose[1]
            
        #get final vector
        self.final_vector = self.goal_vector + self.barrier_vector + self.subgoal_vector
        
        return self.final_vector
                
        
