import random
import matplotlib.pyplot as plt
import numpy as np
import catalina
from astarSingleAUV import singleAUV 
from motion_plan_state import Motion_plan_state
from itertools import permutations

class multiAUV:
    def __init__(self, numAUV, start, habitatList, boundaryList, obstacleList):
        # self.start = start
        self.multiAUV_habitat_open_list = habitatList[:]
        self.multiAUV_habitat_closed_list = []
        self.start = start
        self.boundary_list = boundaryList
        self.obstacle_list = obstacleList
        self.numAUV = numAUV
        self.trajectories = [] # a list holds multiple AUV trajectories
        self.costs = [] # a list holds multiple cost values of different trajectories
    
    def multi_AUV_planner_fixed_start(self, pathLenLimit, weights, start_position_list):
        """
        Find the optimal path for each AUV with fixed start positions  
        Parameter: 
            pathLenLimit: in meters; the length limit of the A* trajectory 
            weights: a list of three numbers [w1, w2, w3] 
            shark_traj_list: a list of shark trajectories that are lists of Motion_plan_state objects 
        """        
        # Create environment
        
        environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS)
        habitat_list = environ[3]

        start_pos_permu_list = permutations(start_position_list)

        for AUV_travel_order_list in start_pos_permu_list:
        
            # Reset the habitat coverage 
            # All habitats are open
            self.multiAUV_habitat_open_list = habitat_list[:]
            self.multiAUV_habitat_closed_list = []
            # print ("\n", "Habitat Open: ", len(self.multiAUV_habitat_open_list))
            
            for start in AUV_travel_order_list:
                print ("\n", "STARTING AT: ", start)
                single_AUV = singleAUV(start, self.obstacle_list, self.boundary_list, self.multiAUV_habitat_open_list, self.multiAUV_habitat_closed_list) 

                # plan path for one singleAUV object 
                single_planner = single_AUV.astar(start, pathLenLimit, weights)
                self.trajectories.append(single_planner["path"])
                print("\n", "Traj: ", single_planner["path"])
                self.costs.append(single_planner["cost"])

                # update overall habitat coverage 
                self.multiAUV_habitat_open_list = single_AUV.habitat_open_list[:]
                self.multiAUV_habitat_closed_list = single_AUV.habitat_closed_list[:]
                # print ("\n", "Habitat Open: ", len(self.multiAUV_habitat_open_list))

                # break
            
        return {"trajs" : self.trajectories, "costs" : self.costs}        

    def multi_AUV_planner(self, pathLenLimit, weights):
        """
        Find the optimal path for each AUV with randomized start positions 
        Parameter: 
            pathLenLimit: in meters; the length limit of the A* trajectory 
            weights: a list of three numbers [w1, w2, w3] 
            shark_traj_list: a list of shark trajectories that are lists of Motion_plan_state objects 
        """
        # List holds number of habitats covered corresponding to the number of AUVs in disposal
        count_habitat_visited = []
        total_habitat_visited = 0 
        for i in range(self.numAUV):
            # print ("start at: ", self.start)
            single_AUV = singleAUV(self.start, self.obstacle_list, self.boundary_list, self.multiAUV_habitat_open_list, self.multiAUV_habitat_closed_list) 

            # plan path for one singleAUV object 
            single_planner = single_AUV.astar(self.start, pathLenLimit, weights)
            self.trajectories.append(single_planner["path"])
            self.costs.append(single_planner["cost"])

            # update overall habitat coverage 
            self.multiAUV_habitat_open_list = single_AUV.habitat_open_list[:]
            self.multiAUV_habitat_closed_list = single_AUV.habitat_closed_list[:]
            total_habitat_visited += len(self.multiAUV_habitat_closed_list)
            count_habitat_visited.append(total_habitat_visited)
            
            # print ("\n", "MultiAUV Open Habitats ", i+1, ": ", self.multiAUV_habitat_open_list)
            # print ("\n", "MultiAUV Closed Habitats ", i+1, ": ", self.multiAUV_habitat_closed_list)

            # print ("\n", "path ", i+1, ": ", single_planner["path"])
            # print ("\n", "path length ", i+1, ": ", len(single_planner["path"]))
            # print ("\n", "path cost ", i+1, ": ", single_planner["cost"])

            del single_AUV
        return {"trajs" : self.trajectories, "costs" : self.costs, "habitats" : count_habitat_visited}

# if __name__ == "__main__":
#     numAUV = 2
#     pathLenLimit = 200
#     weights = [0, 10, 10]
    
#     start_cartesian = catalina.create_cartesian((33.446019, -118.489441), catalina.ORIGIN_BOUND)
#     start = (round(start_cartesian[0], 2), round(start_cartesian[1], 2))

#     environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS)
#     obstacle_list = environ[0] + environ[2]
#     boundary_list = environ[1]
#     habitat_list = environ[3]

#     AUVs = multiAUV(numAUV, habitat_list, boundary_list, obstacle_list)
#     AUVs.multi_AUV_planner(pathLenLimit, weights)