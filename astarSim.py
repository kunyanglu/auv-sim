import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv
import time
import timeit
import random

# Import 3 data representation class
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
        ang -- floating point number, angle in radians

    Return:
        ang -- Updated angle between the range of -pi to pi
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
        self.x = init_x
        self.y = init_y
        self.z = init_z
        self.weights = weights
        self.pathLenLimit = pathLenLimit 
        self.live_graph = Live3DGraph()

    def create_trajectory_list(self, traj_list):
        """
        Update the trajectory list by including the intermediate positions 

        Parameters:
            traj_list -- a list of Motion_plan_state, represent positions of each node

        Returns:
            traj_list -- updated trajectory list
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
        Visualize the planned trajectory

        Parameter:
            NONE

        Returns:
            None
        """

        start = (self.x, self.y)
        # Create the environment
        environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS) 
        obstacle_list = environ[0]
        boundary_list = environ[1]+environ[2]
        habitat_list = environ[3]
        # Create a single_AUV object
        single_AUV = singleAUV(start, obstacle_list, boundary_list, habitat_list, [])
        # Plan path for the single_AUV object
        final_path_mps = single_AUV.astar(habitat_list, obstacle_list, boundary_list, start, self.pathLenLimit, self.weights)
        A_star_traj = final_path_mps["path"]
        A_star_traj_cost = final_path_mps["cost"]

        # PRINT statements
        print ("\n", "trajectory cost: ", A_star_traj_cost)
        print ("\n", "Trajectory: ", A_star_traj)
        print ("\n", "Open Habitats: ", single_AUV.habitat_open_list)
        print ("\n", "Closed Habitats: ", single_AUV.habitat_closed_list)

        # Round the positions of the trajectory
        astar_x_array = []
        astar_y_array = []
        for point in A_star_traj:
            astar_x_array.append(round(point.x, 2))
            astar_y_array.append(round(point.y, 2))
        # Visualize the trajectory
        self.live_graph.plot_2d_astar_traj(astar_x_array, astar_y_array, A_star_traj_cost)

    def collision_free(self, start_pos, obstacle_list):
        """
        Check if the input position collide with any of the obstacles

        Parameters:
            start_pos -- a position tuple (x, y)
            obstacle_list -- a list of Motion_plan_state objects, obstacles
        
        Returns:
            True if start_pos does not collide with obstacle_list
            False if start_pos collides with obstacle_list
        """

        for obstacle in obstacle_list:
            dx = abs(start_pos[0] - obstacle.x)
            dy = abs(start_pos[1] - obstacle.y)
            dist = math.sqrt(dx**2 + dy**2)
            if dist <= obstacle.size:
                return False
        return True
        
    def point_in_triangle(self, p, a, b, c):
        """
        Check if p is inside the triangle formed by connecting a, b, c

        Parameters:
            p -- a position tuple (x, y)
            a -- a position tuple (x, y)
            b -- a position tuple (x, y)
            c -- a position tuple (x, y)

        Returns:
            True if p is inside the triangle
            False if p is outside of the triangle
        """
        if self.same_side(p, a, b, c) and self.same_side(p, b, a, c) and self.same_side(p, c, a, b):
            return True
        else:
            return False

    def same_side(self, p1, p2, a, b):
        """
        Check if p1 and p2 are on the same side

        Parameters:
            p1 -- a position tuple (x, y)
            p2 -- a position tuple (x, y)
            a -- a position tuple (x, y)
            b -- a position tuple (x, y)
        
        Returns:
            True if p1 and p2 are on the same side
            False if p1 and p2 are on different sides
        """

        cp1 = np.cross(np.asarray(b)-np.asarray(a), np.asarray(p1)-np.asarray(a))
        cp2 = np.cross(np.asarray(b)-np.asarray(a), np.asarray(p2)-np.asarray(a))
        if np.dot(cp1, cp2) >= 0:
            return True
        else:
            return False

    def within_bounds(self, boundary_list, position):
        """
        Check if the input position is within the boundary defined by the boundary_list

        Paramters: 
            boundary_list -- a list of Motion_plan_state objects that define the corners of the ROI
            position -- a position tuple (x, y) 

        Returns:
            True if the position is inside the bounds
            False if the position is outside of the bounds
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
        return False 

    def generate_random_start_pos(self): 
        """
        Generate a random starting position within bounds and collision-free

        Parameters:
            NONE
        
        Returns:
            A position tuple (x, y)
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
        Visualize multiple trajectories

        Parameter:
            numAUV -- the number of AUVs
        
        Returns:
            NONE
        """
        # Generate a random starting position
        random_start_pos = self.generate_random_start_pos()
        print ("start at ", random_start_pos)
        # Create the environment
        environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS) 
        obstacle_list = environ[0]
        boundary_list = environ[1] + environ[2]
        habitat_list = environ[3]
        # Create a multi_AUV object
        AUVS = multiAUV(numAUV, random_start_pos, habitat_list, boundary_list, obstacle_list)
        # Plan multiple paths for the multi_AUV object
        multi_AUV = AUVS.multi_AUV_planner(self.pathLenLimit, self.weights)
        # Create Lists for plotting 
        multi_paths = multi_AUV["trajs"]
        multi_costs = multi_AUV["costs"]
        # List holds the habitats covered account corresponding to the number of AUVs in disposal
        multi_habitats = multi_AUV["habitats"]
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
    Generate a plot of the number of AUVs vs. the number of closed habitats

    Parameter:
        None
    
    Returns:
        None
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
    # Plot
    fig, ax = plt.subplots()
    ax.plot(numAUV_list, aveHabitat)
    ax.set(xlabel='AUV Number', ylabel='Number of Habitats Visited', title='Number of AUVs vs. Number of Visited Habitats')
    ax.grid()
    plt.show()

def plot_numAUV_cost():
    """
    Generate a plot of the number of AUVs vs. trajectory cost

    Parameter:
        NONE

    Returns:
        NONE
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
    aveCost = [item/numTrials for item in sumCost]
    numAUV_list = list(range(1, maxNumAUV+1))
    print("numAUV_list: ", numAUV_list)
    print("aveCost: ", aveCost)
    fig, ax = plt.subplots()
    ax.plot(numAUV_list, aveCost)
    ax.set(xlabel='AUV Number', ylabel='Trajectory Cost', title='Number of AUVs vs. Average Trajectory Cost')
    ax.grid()
    plt.show()

def main():
    # Visualize trajectories
    numAUV = 4
    pos = create_cartesian((33.446019, -118.489441), catalina.ORIGIN_BOUND)
    test_robot = astarSim(round(pos[0], 2), round(pos[1], 2), 0, pathLenLimit=250, weights=[10, 1000, 100])
    test_robot.display_multi_astar_trajectory(numAUV)

if __name__ == "__main__":
    main()


