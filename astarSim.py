import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv
import time
import timeit

# import 3 data representation class
from sharkState import SharkState
from sharkTrajectory import SharkTrajectory
from live3DGraph import Live3DGraph
from motion_plan_state import Motion_plan_state

from astarSingleAUV import singleAUV 
from astarMultiAUV import multiAUV 

# from path_planning.rrt_dubins import RRT
from path_planning.cost import Cost
from path_planning.catalina import create_cartesian

# keep all the constants in the constants.py file
# to get access to a constant, eg:
#   const.SIM_TIME_INTERVAL
import constants as const
import path_planning.catalina as catalina

def angle_wrap(ang):
    """
    Takes an angle in radians & sets it between the range of -pi to pi

    Parameter:
        ang - floating point number, angle in radians
    """
    if -math.pi <= ang <= math.pi:
        return ang
    elif ang > math.pi: 
        ang += (-2 * math.pi)
        return angle_wrap(ang)
    elif ang < -math.pi: 
        ang += (2 * math.pi)
        return angle_wrap(ang)


class astarSim:

    def __init__(self, init_x, init_y, init_z, pathLenLimit, weights):
        # initialize auv's data
        self.x = init_x
        self.y = init_y
        self.z = init_z
        self.weights = weights

        self.pathLenLimit = pathLenLimit 
        self.live_graph = Live3DGraph()

    def create_trajectory_list(self, traj_list):
        """
        Run this function to update the trajectory list by including intermediate positions 

        Parameters:
            traj_list - a list of Motion_plan_state, represent positions of each node
        """
        time_stamp = 0.1
        trajectory_list = []

        step = 0 

        for i in range(len(traj_list)-1):
            
            trajectory_list.append(traj_list[i])

            x1 = traj_list[i].x
            y1 = traj_list[i].y
            x2 = traj_list[i+1].x 
            y2 = traj_list[i+1].y

            dx = abs(x1 - x2)
            dy = abs(y1 - y2)
            dist = math.sqrt(dx**2 + dy**2)

            velocity = 1

            time = dist/velocity
            
            counter = 0 

            while counter < time:
                if x1 < x2:
                    x1 += time_stamp * velocity
                if y1 < y2:
                    y1 += time_stamp * velocity
                
                step += time_stamp
                counter += time_stamp
                trajectory_list.append(Motion_plan_state(x1, y1, traj_time_stamp=step))
                
            trajectory_list.append(traj_list[i+1])
            
        return trajectory_list


    def display_single_astar_trajectory(self):
        """
        Display the 2d auv trajectory constructed with A* algorithm

        Parameter:
            NONE 
        """
        start = (self.x, self.y)

        environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS) 
        
        obstacle_list = environ[0]
        boundary_list = environ[1]+environ[2]
        habitat_list = environ[3]

        single_AUV = singleAUV(start, obstacle_list, boundary_list, habitat_list, [])
        final_path_mps = single_AUV.astar(habitat_list, obstacle_list, boundary_list, start, self.pathLenLimit, self.weights)

        A_star_traj = final_path_mps["path"]
        A_star_traj_cost = final_path_mps["cost"]

        print ("\n", "trajectory cost: ", A_star_traj_cost)
        print ("\n", "Trajectory: ", A_star_traj)
        print ("\n", "Open Habitats: ", single_AUV.habitat_open_list)
        print ("\n", "Closed Habitats: ", single_AUV.habitat_closed_list)

        astar_x_array = []
        astar_y_array = []

        for point in A_star_traj:
            astar_x_array.append(round(point.x, 2))
            astar_y_array.append(round(point.y, 2))

        self.live_graph.plot_2d_astar_traj(astar_x_array, astar_y_array, A_star_traj_cost)

    def display_multi_astar_trajectory(self):
        """
        Display multiple A* trajectories

        Parameter:
            None
        """
        start = (self.x, self.y)

        environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS) 
        
        obstacle_list = environ[0]
        boundary_list = environ[1]+environ[2]
        habitat_list = environ[3]    

        AUVS = multiAUV(start, 2, habitat_list, boundary_list, obstacle_list)
        multi_AUV = AUVS.multi_AUV_planner(self.pathLenLimit, self.weights)

        multi_paths = multi_AUV["trajs"]
        multi_costs = multi_AUV["costs"]
        print ("\n", "multi_costs length: ", len(multi_costs))

        X_list = [] # list that holds numAUV-lists of X positions of the trajectory
        Y_list = [] # list that holds numAUV-lists of Y positions of the trajectory
        for path in multi_paths:
            astar_x_array = []
            astar_y_array = []
            for point in path:
                astar_x_array.append(round(point.x, 2))
                astar_y_array.append(round(point.y, 2))
            X_list.append(astar_x_array)
            Y_list.append(astar_y_array)
        print ("\n", "X_list length: ", len(X_list))
        self.live_graph.plot_multiple_2d_astar_traj(X_list, Y_list, multi_costs)

def main():
    pos = create_cartesian((33.446019, -118.489441), catalina.ORIGIN_BOUND)
    test_robot = astarSim(round(pos[0], 2), round(pos[1], 2), 0, pathLenLimit=1000, weights=[0, 10, 10])
    test_robot.display_multi_astar_trajectory()
    # test_robot.display_single_astar_trajectory()

if __name__ == "__main__":
    main()
