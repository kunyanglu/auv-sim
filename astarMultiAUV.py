import random
import matplotlib.pyplot as plt
import numpy as np
import path_planning.catalina as catalina
from astarSingleAUV import singleAUV 
from motion_plan_state import Motion_plan_state

class multiAUV:
    def __init__(self, numAUV, habitatList, boundaryList, obstacleList):
        # self.start = start
        self.habitat_open_list = habitatList[:]
        self.habitat_closed_list = []
        self.boundary_list = boundaryList
        self.obstacle_list = obstacleList
        self.numAUV = numAUV
        self.trajectories = [] # a list holds multiple AUV trajectories
        self.costs = [] # a list holds multiple cost values of different trajectories

    def multi_AUV_planner(self, pathLenLimit, weights):
        """
        Find the optimal path for each AUV  
        Parameter: 
            pathLenLimit: in meters; the length limit of the A* trajectory 
            weights: a list of three numbers [w1, w2, w3] 
            shark_traj_list: a list of shark trajectories that are lists of Motion_plan_state objects 
        """

        for i in range(self.numAUV):
            # create singleAUV object 
            random_start_position = (random.randint(-400, 0), random.randint(-100, 100))
            print ("\n", "staring position ", i, ": ", random_start_position)
            single_AUV = singleAUV(random_start_position, self.obstacle_list, self.boundary_list, self.habitat_open_list, self.habitat_closed_list) 
        
            # plan path for one singleAUV object 
            single_planner = single_AUV.astar(self.habitat_open_list, self.obstacle_list, self.boundary_list, random_start_position, pathLenLimit, weights)
            self.trajectories.append(single_planner["path"])
            self.costs.append(single_planner["cost"])

            # update overall abitat coverage 
            self.habitat_open_list = single_AUV.habitat_open_list[:]
            self.habitat_closed_list = single_AUV.habitat_closed_list[:]

            print ("\n", "Open Habitats ", i+1, ": ", self.habitat_open_list)
            print ("\n", "Closed Habitats ", i+1, ": ", self.habitat_closed_list)
            print ("\n", "path ", i+1, ": ", single_planner["path"])
            print ("\n", "path length ", i+1, ": ", len(single_planner["path"]))
            print ("\n", "path cost ", i+1, ": ", single_planner["cost"])

            del single_AUV
            
        return {"trajs" : self.trajectories, "costs" : self.costs}

if __name__ == "__main__":
    numAUV = 2
    pathLenLimit = 200
    weights = [0, 10, 10]
    
    start_cartesian = catalina.create_cartesian((33.446019, -118.489441), catalina.ORIGIN_BOUND)
    start = (round(start_cartesian[0], 2), round(start_cartesian[1], 2))

    environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS)
    obstacle_list = environ[0] + environ[2]
    boundary_list = environ[1]
    habitat_list = environ[3]

    AUVs = multiAUV(start, numAUV, habitat_list, boundary_list, obstacle_list)
    AUVs.multi_AUV_planner(pathLenLimit, weights)