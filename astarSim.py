import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv
import time
import timeit
import random

# import 3 data representation class
from sharkState import SharkState
from sharkTrajectory import SharkTrajectory
from live3DGraph import Live3DGraph
from itertools import permutations
from motion_plan_state import Motion_plan_state

from astarSingleAUV import singleAUV 
from astarMultiAUV import multiAUV 

# from path_planning.rrt_dubins import RRT
from path_planning.cost import Cost
from catalina import create_cartesian

from shapely.wkt import loads as load_wkt 
from shapely.geometry import Polygon
from operator import add

# keep all the constants in the constants.py file
# to get access to a constant, eg:
#   const.SIM_TIME_INTERVAL
import constants as const
import catalina

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

    def collision_free(self, start_pos, obstacle_list):
        """
        Return true if start_pos does not collide with any obstacle
        otherwise return false
        """

        for obstacle in obstacle_list:
            dx = abs(start_pos[0] - obstacle.x)
            dy = abs(start_pos[1] - obstacle.y)
            dist = math.sqrt(dx**2 + dy**2)
            if dist <= obstacle.size:
                return False
        return True
        
    def point_in_triangle(self, p, a, b, c):
        if self.same_side(p, a, b, c) and self.same_side(p, b, a, c) and self.same_side(p, c, a, b):
            return True
        else:
            return False

    def same_side(self, p1, p2, a, b):
        cp1 = np.cross(np.asarray(b)-np.asarray(a), np.asarray(p1)-np.asarray(a))
        cp2 = np.cross(np.asarray(b)-np.asarray(a), np.asarray(p2)-np.asarray(a))
        if np.dot(cp1, cp2) >= 0:
            return True
        else:
            return False

    def within_bounds(self, boundary_list, position):
        """
        Check if the given position is within the given booundry expressed as a polygon
        Paramter: 
            boundary_list: a list of Motion_plan_state objects that define the corners of the region of interest
            position: a tuple of two elements (x, y) to test whether it's within the boundary or not 
        """
        poly_list = []  
        for corner in boundary_list: 
            poly_list.append([corner.x, corner.y])

        centroid = Polygon(poly_list).centroid.coords
        for index in range(len(poly_list)):
            if index != len(poly_list)-1:
                if self.point_in_triangle(position, poly_list[index], poly_list[index+1], centroid):
                    return True 
            else:
                if self.point_in_triangle(position, poly_list[len(poly_list)-1], poly_list[0], centroid):
                    return True
        return False # Not within boundary

    def generate_random_start_pos(self): 
        """"
        Generate a random start position 
        that does not collide with obstacles and within bounds
        """
        # Set up the environment
        environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS)
        obstacle_list = environ[0] + environ[2]
        boundary_list = environ[1]  
        # Initialize doable to Flase
        doable = False
        while doable == False:
            random_start_position = (random.randint(-500, 100), random.randint(-200, 200))
            # check if no collision
            if self.collision_free(random_start_position, obstacle_list):
                # check if within bounds
                if self.within_bounds(boundary_list, random_start_position):
                    doable = True
                    return random_start_position
        if doable == False:
            return (-100,0)

    def display_multi_astar_trajectory(self, numAUV):
        """
        Display multiple A* trajectories

        Parameter:
            None
        """
        # Turn on switch if we want to visualize trajectories, otherwise turn it off
        # switch = True 
        random_start_pos = self.generate_random_start_pos()
        print ("start at ", random_start_pos)
        environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS) 
        
        obstacle_list = environ[0]
        boundary_list = environ[1] + environ[2]
        habitat_list = environ[3]

        AUVS = multiAUV(numAUV, random_start_pos, habitat_list, boundary_list, obstacle_list)
        # Call multi_AUV with randomized starting positions 
        multi_AUV = AUVS.multi_AUV_planner(self.pathLenLimit, self.weights)
        
        # Call multi_AUV with fixed starting positions 
        # multi_AUV = AUVS.multi_AUV_planner_fixed_start(self.pathLenLimit, self.weights, starting_position_list)

        multi_paths = multi_AUV["trajs"]
        multi_costs = multi_AUV["costs"]
        # List holds number of habitats covered corresponding to the number of AUVs in disposal
        multi_habitats = multi_AUV["habitats"] 

        # return multi_habitats
        # if switch == False:
        #     # turn on if we chose to visualize trajectories
        #     print("cost: ", multi_costs)
        #     return multi_costs # [1st AUV cost, 2nd AUV cost, ...]

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
        
        # Visualize the trajectories
        self.live_graph.plot_multiple_2d_astar_traj(X_list, Y_list, multi_costs)

def plot_numAUV_numHabitat():
    """
    Generate a Plot of the number of AUVs vs. number of habitats explored
    """
    robot = astarSim(0, 0, 0, pathLenLimit=100, weights=[0, 10, 1000])
    maxNumAUV = 10
    numTrials = 20
    sumHabitats = [0] * maxNumAUV
    currTrial = 1
    while currTrial < numTrials:
        print("Trial: ", currTrial)
        habitats = robot.display_multi_astar_trajectory(maxNumAUV)
        sumHabitats = list(map(add, sumHabitats, habitats))
        print ("sumHabitats: ", sumHabitats)
        currTrial += 1
        print(end='')
    aveHabitat = [item/numTrials for item in sumHabitats] # Y axis
    numAUV_list = list(range(1, maxNumAUV+1)) # X axis
    print("numAUV_list: ", numAUV_list)
    print("aveHabitat: ", aveHabitat)

    fig, ax = plt.subplots()
    ax.plot(numAUV_list, aveHabitat)
    ax.set(xlabel='AUV Number', ylabel='Number of Habitats Visited', title='Number of AUVs vs. Number of Visited Habitats')
    ax.grid()
    plt.show()

def plot_numAUV_cost():
    """
    Generate Plot of number of AUVs vs. Trajectory Cost
    """
    robot = astarSim(0, 0, 0, pathLenLimit=100, weights=[0, 10, 1000])
    maxNumAUV = 10
    numTrials = 20
    sumCost = [0] * maxNumAUV

    currTrial = 1
    while currTrial < numTrials:
        print("Trial: ", currTrial)
        cost = robot.display_multi_astar_trajectory(maxNumAUV)
        sumCost = list(map(add, sumCost, cost))
        print("sumCost: ", sumCost)
        currTrial += 1
        print(end='')
    
    aveCost = [item/numTrials for item in sumCost] # Y axis
    numAUV_list = list(range(1, maxNumAUV+1)) # X axis
    print("numAUV_list: ", numAUV_list)
    print("aveCost: ", aveCost)

    fig, ax = plt.subplots()
    ax.plot(numAUV_list, aveCost)
    ax.set(xlabel='AUV Number', ylabel='Trajectory Cost', title='Number of AUVs vs. Average Trajectory Cost')
    ax.grid()
    plt.show()

def main():
    # Visualize trajectories
    numAUV = 2
    pos = create_cartesian((33.446019, -118.489441), catalina.ORIGIN_BOUND)
    test_robot = astarSim(round(pos[0], 2), round(pos[1], 2), 0, pathLenLimit=500, weights=[0, 10, 1000])
    test_robot.display_multi_astar_trajectory(numAUV)

    # Plot numAUV vs. Cost
    # plot_numAUV_cost()
    # test_robot.display_single_astar_trajectory()
if __name__ == "__main__":
    # plot_numAUV_numHabitat()
    main()


