import math
import geopy.distance
import random
import timeit
from motion_plan_state import Motion_plan_state
import matplotlib.pyplot as plt
import numpy as np
import catalina

from shapely.wkt import loads as load_wkt 
from shapely.geometry import Polygon 

from path_planning.catalina import create_cartesian
from cost import Cost

def euclidean_dist(point1, point2):
    """
    Calculate the squared distance between the two points

    Parameters:
        point1 - a position tuple: (x, y)
        point2 - a position tuple: (x, y)
    
    Returns:
        The distance between the two points
    """

    dx = abs(point1[0]-point2[0])
    dy = abs(point1[1]-point2[1])

    return math.sqrt(dx*dx+dy*dy)

class Node: 
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0 
        self.h = 0
        self.f = 0  
        self.cost = 0
        self.pathLen = 0 # The current path length
        self.time_stamp = 0

class singleAUV:
    def __init__(self, start, obstacleList, boundaryList, habitatOpenList, habitatClosedList):
        self.path = [] # a list of motion plan state 
        self.start = start
        self.obstacle_list = obstacleList
        self.boundary_list = boundaryList
        self.habitat_open_list = habitatOpenList[:]
        self.habitat_closed_list = habitatClosedList[:]
        self.visited_nodes = np.zeros([600, 600])
        self.velocity = 1 
        
    def euclidean_dist(self, point1, point2): # point is a position tuple (x, y)
        """
        Calculate the distance square between two points

        Parameters:
            point1 -- a position tuple: (x, y)
            point2 -- a position tuple: (x, y)

        Returns:
            Distane in meters
        """

        dx = abs(point1[0]-point2[0])
        dy = abs(point1[1]-point2[1])

        return dx*dx+dy*dy
    
    def get_distance_angle(self, start_mps, end_mps):
        """
        Calculate the distance and angle between the two points

        Parameters:
            start_mps -- a Motion_plan_state object
            end_mps -- a Motion_plan_state object

        Returns:
            Distance and angle between the two points
        """

        dx = end_mps.x-start_mps.x
        dy = end_mps.y-start_mps.y
        dist = math.sqrt(dx**2 + dy**2)
        theta = math.atan2(dy,dx)
        return dist, theta

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
        
    def collision_free(self, position, obstacleList): # obstacleList lists of motion plan state 
        """
        Check if the input position collide with any of the obstacles

        Parameters:
            start_pos -- a position tuple (x, y)
            obstacle_list -- a list of Motion_plan_state objects, obstacles
        
        Returns:
            True if start_pos does not collide with obstacle_list
            False if start_pos collides with obstacle_list
        """

        position_mps = Motion_plan_state(position[0], position[1])
        for obstacle in obstacleList:
            d, _ = self.get_distance_angle(obstacle, position_mps)
            if d <= obstacle.size:
                return False
        return True 

    def Walkable(self, check_point, current_point, obstacleList):
        """
        Check if the distance from the current_point to the check_point is collision free

        Parameters:
            check_point -- a Motion_plan_state object
            current_point -- a Motion_plan_state object
            obstacleList -- a list of Motion_plan_state objects, obstacles
        
        Returns:
            True if there is no obstacle between the current_point and the check_point
            False if there is obstacle between the current_point and the check_point
        """

        start_x = check_point.x
        start_y = check_point.y
        step_x = int(abs(current_point.x - check_point.x)/5)
        step_y = int(abs(current_point.y - check_point.y)/5)
        while start_x <= current_point.x and start_y <= current_point.y:
            intermediate = (start_x, start_y)
            start_x += step_x
            start_y += step_y
            if not self.collision_free(intermediate, obstacleList):
                return False
        return True

    def curr_neighbors(self, current_node, boundary_list): 

        """
        Create a list of positions tuples that are neighbors of the current point

        Parameters:
            current_node -- a Node object
            boundary_list -- a list of Motion_plan_state objects

        Returns:
            A list of positions tuples that are neighbors of the current point
        """

        adjacent_squares = [(0, -10), (0, 10), (-10, 0), (10, 0), (-10, -10), (-10, 10), (10, -10), (10, 10)]
        current_neighbors = []
        for new_position in adjacent_squares:
            node_position = (current_node.position[0]+new_position[0], current_node.position[1]+new_position[1])
            # Check if the node is within the bounds
            if self.within_bounds(boundary_list, node_position):
                current_neighbors.append(node_position)
        return current_neighbors  

    def habitats_time_spent(self, current_node):
        """
        Get the approximate time spent in habitats if the current node is within the habitat(s)
        
        Parameters:
            current_node -- a position tuple (x,y)
    
        Returns:
            Time spent in habitats
        """

        dist_in_habitats = 0
        velocity = 1
        for habi in catalina.HABITATS:
            pos_habi = create_cartesian((habi.x, habi.y), catalina.ORIGIN_BOUND)
            dist, _ = self.get_distance_angle(Motion_plan_state(pos_habi[0], pos_habi[1], size=habi.size), Motion_plan_state(current_node.position[0], current_node.position[1]))
            if dist <= habi.size:
                dist_in_habitats += 10
                
        return dist_in_habitats/velocity
    
    def update_habitat_coverage(self, current_node, habitat_open_list, habitat_closed_list):
        """
        Update the habitat_open_list and habitat_closed_list if the current_node covers a new habitat
     
        Parameters:
            current_node -- a Node object 
            habitat_open_list -- a list of Motion_plan_state objects, habitats
            habitat_closed_list -- a list of Motion_plan_state objects, habitats

        Returns:
            NONE
        """

        for index, item in enumerate(habitat_open_list):
            dist = math.sqrt((current_node.position[0]-item.x) **2 + (current_node.position[1]-item.y) **2)
            # Check if the current_node covers a new habitat
            if dist <= item.size:
                # Update the habitat coverage
                self.habitat_open_list.pop(index)
                self.habitat_closed_list.append(item)
    
    def inside_habitats(self, mps_to_check, habitats):
        """
        Check is mps is inside any of the habitats (closed and open habitats)

        Parameters:
            mps_to_check -- a Motion_plan_state object
            habitats -- a list of Motion_plan_state objects, habitats
        
        Returns:
            True if mps_to_check is inside any of the habitats
            False if mps_to_check is outside of all the habitats
        """
    
        for habitat in habitats:
            dist = euclidean_dist((habitat.x, habitat.y), (mps.x, mps. y))
            if dist <= habitat.size:
                return True

        return False
    
    def get_indices(self, x_in_meters, y_in_meters):
        """
        Convert x, y in the coordinate system of the catalina environment to the coordinate system of the visited nodes
        in order to have these two positions correspond to each other 
        Parameter: 
            x_in_meters: a decimal
            y_in_meters: a decimal
        """
        x_pos = int(x_in_meters + 500)
        y_pos = int(y_in_meters + 200)

        return (x_pos, y_pos)

    def smoothPath(self, trajectory, haibitats): 
        """
        Create a smoothed trajectory without the intermediate waypoints

        Parameters: 
            trajectory -- a list of Motion_plan_state objects
            obstacleList -- a list of Motion_plan_state objects
        
        Returns:
            A smoothed trajectory
        """

        index = 0 
        checkPoint = trajectory[index] 
        currentPoint = trajectory[index]
        smoothTraj = trajectory[:] # List holds smoothed trajectory 
        
        while index < len(trajectory)-1:
            # Check if from the checkPoint to currentPoint is collision free
            if self.Walkable(checkPoint, currentPoint, self.obstacle_list):
                inside_habitats = self.inside_habitats(currentPoint, haibitats)
                # If the currentPoint is not inside habitats, remove the currentPoint
                if not inside_habitats:
                    temp = currentPoint
                    index += 1 
                    currentPoint = trajectory[index]                    
                    smoothTraj.remove(temp)
                # If the currentPoint is inside habitats
                else: 
                    index += 1
                    currentPoint = trajectory[index]
            else:
                checkPoint = currentPoint
                index += 1
                currentPoint = trajectory[index]

        return smoothTraj

    def NumHabitatInRange(self, childNode, openHabitats, pathLenLimit):
        """
        Get the number of habitats reachable within path length limit

        Parameters:
            childNode -- a Node object
            openHabitats -- a list of open habitats
            pathLenLimit -- path length limit in meters
        
        Returns:
            The number of habitats reachable within path length limit
        """
        reachableDist = (abs(childNode.pathLen - pathLenLimit))**2
        NumHabitatInRange = 0
        for habitat in openHabitats:
            dx = abs(habitat.x - childNode.position[0])
            dy = abs(habitat.y - childNode.position[1])
            dist = dx**2 + dy**2
            if dist <= reachableDist:
                NumHabitatInRange += 1
        return NumHabitatInRange

    def astar(self, start, pathLenLimit, weights): 
        """
        Find the optimal path from start to goal avoiding given obstacles 
        
        Parameters: 
            start -- a position tuple (x, y)
            pathLenLimit -- the path length limit in meters
            weights -- a list of coefficients [w1, w2, w3]
        
        Returns:
            The optimal trajectory
        """
        # Initialize weights
        weight_10 = weights[0]
        weight_1000 = weights[1]
        weight_100 = weights[2]
        # Initialize time spent in habitats
        habitats_time_spent = 0
        # Initialize cost
        cal_cost = Cost()
        cost = []
        # Initialize start node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        
        open_list = [] # List hold sneighbors of the expanded nodes
        closed_list = [] # List holds all the exapnded nodes
        open_list.append(start_node)
        while len(open_list) > 0:
            current_node = open_list[0] # Initialize the current node
            current_index = 0
            # Find the current node with the smallest f
            for index, item in enumerate(open_list): 
                if item.f < current_node.f:
                    current_node = item
                    current_index = index
            # Update habitat coverage
            self.update_habitat_coverage(current_node, self.habitat_open_list, self.habitat_closed_list)  
            # Update open_list and closed_list
            open_list.pop(current_index)
            closed_list.append(current_node)
            
            # Check if the current path reaches the path length limit 
            if abs(current_node.pathLen - pathLenLimit) <= 10:
                path = []
                current = current_node
                # Backtrack to find the path
                while current is not None: 
                    path.append(current.position)
                    cost.append(current.cost)
                    habitats_time_spent += self.habitats_time_spent(current)
                    current = current.parent
                # Make the path a list of Motion_plan_state objects
                path_mps = []
                for point in path:
                    mps = Motion_plan_state(point[0], point[1])
                    path_mps.append(mps)
                trajectory = path_mps[::-1]
                # Smooth the trajectory
                # smoothPath = self.smoothPath(trajectory, habitats)
                return ({"path" : trajectory, "cost list" : cost, "cost" : cost[0]})
            # Find the neighbors of the current node
            current_neighbors = self.curr_neighbors(current_node, self.boundary_list)
            children = []
            
            # Make the neighbor the child node of the current node if collision-free
            for neighbor in current_neighbors:
                if self.collision_free(neighbor, self.obstacle_list):
                    new_node = Node(current_node, neighbor)
                    children.append(new_node)
            
            for child in children:
                # Continue to beginning of for loop if child is in closed list
                if child in closed_list:
                    continue
                # Compute the attributes if the child node is new
                habitatInfo = cal_cost.cost_of_edge(child, self.habitat_open_list, self.habitat_closed_list, weights) 
                insideClosedHabitats = habitatInfo[1]
                insideOpenHabitats = habitatInfo[2]
                child.g = child.parent.cost - weight_10 * insideClosedHabitats - weight_1000 * insideOpenHabitats 
                child.cost = child.g
                NumHabitatInRange = self.NumHabitatInRange(child, self.habitat_open_list, pathLenLimit)
                child.h = - weight_10 * abs(pathLenLimit - child.pathLen) - weight_100 * NumHabitatInRange
                child.f = child.g + child.h 
                child.pathLen = child.parent.pathLen + euclidean_dist(child.parent.position, child.position)
                child.time_stamp = int(child.pathLen/self.velocity)
                # Child is already in the open list
                # Get the child node that has g smaller than the g of the open node
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue
                x_pos, y_pos = self.get_indices(child.position[0], child.position[1])
                # Update the map
                if self.visited_nodes[x_pos, y_pos] == 0: 
                    open_list.append(child)
                    self.visited_nodes[x_pos, y_pos] = 1

# def main():
    # weights = [0, 10, 1000]
    # start_cartesian = create_cartesian((33.445089, -118.486933), catalina.ORIGIN_BOUND)
    # start = (round(start_cartesian[0], 2), round(start_cartesian[1], 2))
    # print ("start: ", start) 
    # #  convert to environment in casrtesian coordinates 
    # environ = catalina.create_environs(catalina.OBSTACLES, catalina.BOUNDARIES, catalina.BOATS, catalina.HABITATS)
    # obstacle_list = environ[0]
    # boundary_list = environ[1]+environ[2]
    # habitat_list = environ[3] 
    # single_AUV = singleAUV(start, obstacle_list, boundary_list, habitat_list, []) 
    # final_traj = single_AUV.astar(start, 800, weights)
    # print ("\n", "final trajectory: ",  final_traj["path"])
    # print ("\n", "Trajectory length: ", len(final_traj["path"]))
    # print ("\n", "cost of each node on the final trajectory: ",  final_traj["cost"])       

# if __name__ == "__main__":
#     main()