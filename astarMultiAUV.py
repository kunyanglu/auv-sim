import random
import matplotlib.pyplot as plt
import numpy as np
import catalina
from astarSingleAUV import singleAUV 
from motion_plan_state import Motion_plan_state
from itertools import permutations

class multiAUV:
    def __init__(self, numAUV, start, habitatList, boundaryList, obstacleList):
       
        self.multiAUV_habitat_open_list = habitatList[:]
        self.multiAUV_habitat_closed_list = []
        self.start = start
        self.boundary_list = boundaryList
        self.obstacle_list = obstacleList
        self.numAUV = numAUV
        self.trajectories = [] # multiple AUV trajectories
        self.costs = [] # multiple cost values corresponding to each trajectory
    
    def multi_AUV_planner_fixed_start(self, pathLenLimit, weights, start_position_list):
        """
        Find the optimal path for each AUV with fixed start positions

        Parameter: 
            pathLenLimit -- the length limit of the planned trajectory in meters
            weights -- a list of three coefficients [w1, w2, w3] 
            start_position_list -- a list of starting positions (x, y)

        Returns:
            dict -- a dictionary with "traj" matches a list of trajectory lists, 
            "costs" matches a list of costs corresponding to each trajectory list
        """    

        # Create environment  
        environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS)
        habitat_list = environ[3]
        # Create permutations of the starting position list
        start_pos_permu_list = permutations(start_position_list)

        for AUV_travel_order_list in start_pos_permu_list:
           
            self.multiAUV_habitat_open_list = habitat_list[:] # Reset the habitat coverage 
            self.multiAUV_habitat_closed_list = [] # All habitats are open
            
            for start in AUV_travel_order_list:
                print ("\n", "STARTING AT: ", start)
                # Create a new object, single_AUV 
                single_AUV = singleAUV(start, self.obstacle_list, self.boundary_list, self.multiAUV_habitat_open_list, self.multiAUV_habitat_closed_list) 
                # Plan path for the single_AUV object 
                single_planner = single_AUV.astar(start, pathLenLimit, weights)
                # Append the new path
                self.trajectories.append(single_planner["path"])
                # Append the new cost
                self.costs.append(single_planner["cost"])
                print("\n", "Traj: ", single_planner["path"])

                # Update the overall habitat coverage 
                self.multiAUV_habitat_open_list = single_AUV.habitat_open_list[:]
                self.multiAUV_habitat_closed_list = single_AUV.habitat_closed_list[:]

        return {"trajs" : self.trajectories, "costs" : self.costs}        

    def multi_AUV_planner(self, pathLenLimit, weights):
        """
        Find the optimal path for each AUV with randomized starting positions 

        Parameters: 
            pathLenLimit -- the length limit of the planned trajectory in meters
            weights -- a list of three coefficients [w1, w2, w3]

        Return:
            dict -- a dictionary with "traj" matches a list of trajectory lists, 
            "costs" matches a list of costs corresponding to each trajectory list            
        """
<<<<<<< HEAD
        # List holds number of habitats covered corresponding to the number of AUVs in disposal
        count_habitat_visited = []
        total_habitat_visited = 0 

        for i in range(self.numAUV):
        
=======

        count_habitat_visited = [] # A list of the number of habitats that each AUV visited
        total_habitat_visited = 0  # The total number of habitats visited by all AUVs

        for i in range(self.numAUV):
            print ('\n', "AUV", i+1)
            # Create a new object, single_AUV 
>>>>>>> 96f4bdb0ee8c8c20b7f0c4becf194855f19cc5a6
            single_AUV = singleAUV(self.start, self.obstacle_list, self.boundary_list, self.multiAUV_habitat_open_list, self.multiAUV_habitat_closed_list) 
            # Plan path for the single_AUV object 
            single_planner = single_AUV.astar(self.start, pathLenLimit, weights)
            # Append the new path
            self.trajectories.append(single_planner["path"])
            # Append the new cost
            self.costs.append(single_planner["cost"])

            # Update the overall habitat coverage 
            self.multiAUV_habitat_open_list = single_AUV.habitat_open_list[:]
            self.multiAUV_habitat_closed_list = single_AUV.habitat_closed_list[:]
            # Update the number of visited habitats
            total_habitat_visited += len(self.multiAUV_habitat_closed_list)
            count_habitat_visited.append(total_habitat_visited)
<<<<<<< HEAD
            
            print ("\n", "MultiAUV Open Habitats", i+1, ": ", len(self.multiAUV_habitat_open_list))
            print ("\n", "MultiAUV Closed Habitats", i+1, ": ", len(self.multiAUV_habitat_closed_list))
=======
            del single_AUV

            # PRINT statements
            # print ("\n", "MultiAUV Open Habitats ", i+1, ": ", self.multiAUV_habitat_open_list)
            # print ("\n", "MultiAUV Closed Habitats ", i+1, ": ", self.multiAUV_habitat_closed_list)
>>>>>>> 96f4bdb0ee8c8c20b7f0c4becf194855f19cc5a6

            # print ("\n", "path ", i+1, ": ", single_planner["path"])
            # print ("\n", "path length ", i+1, ": ", len(single_planner["path"]))
            # print ("\n", "path cost ", i+1, ": ", single_planner["cost"])     
        return {"trajs" : self.trajectories, "costs" : self.costs, "habitats" : count_habitat_visited}
