import math
import networkx as nx
import numpy as np

class PathCompute():
    def __init__(self, map_path):
        self.map_path = map_path
        self.EM = np.loadtxt(map_path + 'EM_weight.txt')
        topo_map = np.loadtxt(map_path + 'em_map.txt')
        self.topo_map = nx.from_numpy_matrix(topo_map)
        self.path = []
        
    def topo_path_compute(self, current_pose, goal):
        n_x = current_pose[0]
        n_y = current_pose[1]
        goal_x = goal[0]
        goal_y = goal[1]
        #compute nearest point in EM_weight
        start_temp_array = []
        goal_temp_array = []
        for EM_point in self.EM:
            start_temp_array.append(math.pow(n_x - EM_point[0], 2) + math.pow(n_y - EM_point[1], 2))
            goal_temp_array.append(math.pow(goal_x - EM_point[0], 2) + math.pow(goal_y - EM_point[1], 2))
        start_point_node = np.argmin(start_temp_array)
        goal_point_node = np.argmin(goal_temp_array)
        print('start_node', start_point_node, 'goal_node', goal_point_node)
        #find shortest way in topo map
        self.path = nx.shortest_path(self.topo_map, source=start_point_node, target=goal_point_node)
        print(self.path)
        return self.path