import math
import random
import time
import csv

import matplotlib.pyplot as plt
from matplotlib import cm, patches, collections
import numpy as np
from shapely.geometry import Polygon, Point

from motion_plan_state import Motion_plan_state
from cost import Cost
import catalina
from sharkOccupancyGrid import SharkOccupancyGrid, splitCell
#from shortest_rrt import Shrt_path

show_animation = True


class RRT:
    """
    Class for RRT planning
    """
    def __init__(self, start, goal, boundary, obstacles, habitats, exp_rate = 1, dist_to_end = 2, diff_max = 0.5, freq = 30):
        '''setting parameters:
            initial_location: initial Motion_plan_state of AUV, [x, y, z, theta, v, w, time_stamp]
            goal_location: Motion_plan_state of the shark, [x, y, z]
            obstacle_list: Motion_plan_state of obstacles [[x1, y1, z1, size1], [x2, y2, z2, size2] ...]
            boundary: max & min Motion_plan_state of the configuration space [[x_min, y_min, z_min],[x_max, y_max, z_max]]'''
        #initialize start, goal, obstacle, boundary, habitats for path planning
        self.start = start
        self.goal = goal
        self.boundary = boundary
        #initialize corners for boundaries
        coords = []
        for corner in self.boundary: 
            coords.append((corner.x, corner.y))
        self.boundary_poly = Polygon(coords)
        self.obstacle_list = obstacles
        #testing data for habitats
        self.habitats = habitats
        
        self.mps_list = [] # a list of motion_plan_state
        self.time_bin = {}

        #if minimum path length is not achieved within maximum iteration, return the latest path
        self.last_path = []

        #setting parameters for path planning
        self.exp_rate = exp_rate
        self.dist_to_end = dist_to_end
        self.diff_max = diff_max
        self.freq = freq

    def exploring(self, shark_dict, sharkGrid, plot_interval, bin_interval, v, shark_interval, traj_time_stamp=False, max_plan_time=5, max_traj_time=200.0, plan_time=True, weights=[1,-1,-1,-1]):
        """
        rrt path planning without setting a specific goal, rather try to explore the configuration space as much as possible
        calculate cost while expand and keep track of the current optimal cost path
        max_iter: maximum iteration for the tree to expand
        plan_time: expand by randomly picking a time stamp and find the motion_plan_state along the path with smallest time difference
        """

        # keep track of the motion_plan_state whose path is optimal
        opt_cost = [float("inf")]
        opt_path = [0,[]]
        opt_cost_list = []
        
        # keep track of longest traj_time_stamp
        # longest_traj_time = 0

        # keep track of the longest single path in the tree to normalize every path length
        peri_boundary = self.cal_boundary_peri()

        # initialize cost function
        cal_cost = Cost()

        # initialize shark occupancy grid
        self.cell_list = splitCell(self.boundary_poly, 10)
        if sharkGrid == {}:
            self.sharkGrid = createSharkGrid('path_planning/AUVGrid_prob.csv', self.cell_list)
        else:
            self.sharkGrid = sharkGrid
        self.sharkDict = shark_dict

        self.mps_list = [self.start]

        self.t_start = time.time()
        n_expand = math.ceil(max_plan_time / plot_interval)

        if traj_time_stamp:
            time_expand = math.ceil(max_traj_time / bin_interval)
            for i in range(1, time_expand + 1):
                self.time_bin[bin_interval * i] = []
            self.time_bin[bin_interval].append(self.start)

        for i in range(1, n_expand + 1):
            t_end = self.t_start + i * plot_interval
            while time.time() < t_end:
                # find the closest motion_plan_state by generating a random time stamp and 
                # find the motion_plan_state whose time stamp is closest to it
                if plan_time:
                    if traj_time_stamp:
                        ran_bin = int(random.uniform(1, time_expand + 1))
                        while self.time_bin[bin_interval * ran_bin] == []:
                            ran_bin = int(random.uniform(1, time_expand + 1))
                        ran_index = int(random.uniform(0, len(self.time_bin[bin_interval * ran_bin])))
                        closest_mps = self.time_bin[bin_interval * ran_bin][ran_index]
                    else:
                        ran_time = random.uniform(0, max_plan_time * self.freq)
                        closest_mps = self.get_closest_mps_time(ran_time, self.mps_list)
                        if closest_mps.traj_time_stamp > max_traj_time:
                            continue
                # find the closest motion_plan_state by generating a random motion_plan_state
                # and find the motion_plan_state with smallest distance
                else:
                    ran_mps = self.get_random_mps()
                    closest_mps = self.get_closest_mps(ran_mps, self.mps_list)
                    if closest_mps.traj_time_stamp > max_traj_time:
                        continue

                new_mps = self.steer(closest_mps, self.dist_to_end, self.diff_max, self.freq, v, traj_time_stamp)  
                
                if self.check_collision(new_mps, self.obstacle_list):
                    new_mps.parent = closest_mps
                    path = self.generate_final_course(new_mps)
                    new_mps.length = self.cal_length(path)
                    self.mps_list.append(new_mps)
                    #add to time stamp bin
                    if traj_time_stamp:
                        curr_bin = (new_mps.traj_time_stamp // bin_interval + 1) * bin_interval
                        if curr_bin > max_traj_time:
                            #continue
                            self.time_bin[curr_bin] = []
                        self.time_bin[curr_bin].append(new_mps)
                    # if new_mps.traj_time_stamp > longest_traj_time:
                    #     longest_traj_time = new_mps.traj_time_stamp
                    #else:
                    #    if new_mps.traj_time_stamp > max_traj_time:
                    #        continue    
                    #Question: how to normalize the path length?
                    if new_mps.length >= 80:
                        #find the corresponding shark occupancy grid
                        sharkOccupancyDict = {}
                        start = closest_mps.traj_time_stamp
                        end = new_mps.traj_time_stamp
                        for time_bin in self.sharkGrid:
                            if (start >= time_bin[0] and start <= time_bin[1]) or (time_bin[0] >= start and time_bin[1] <= end) or(end >= time_bin[0] and end <= time_bin[1]):
                                sharkOccupancyDict[time_bin] = self.sharkGrid[time_bin]
                        
                        temp_length = new_mps.length - closest_mps.length
                        traj_time = new_mps.traj_time_stamp - closest_mps.traj_time_stamp
                        if traj_time != 0:    
                            new_cost = cal_cost.habitat_shark_cost_func(new_mps.path, temp_length, peri_boundary, traj_time, self.habitats, sharkOccupancyDict, weights)
                            if closest_mps.cost == []:
                                new_mps.cost = new_cost
                            else:
                                temp_cost = []
                                for i in range(len(new_cost[1])):
                                    temp_cost.append(closest_mps.cost[1][i]+new_cost[1][i])
                                new_mps.cost = [closest_mps.cost[0]+new_cost[0], temp_cost]
                            if new_mps.cost[0] < opt_cost[0]:
                                opt_cost = new_mps.cost
                                opt_path = [new_mps.length, path]
                
            opt_cost_list.append(opt_cost[0])
        path = self.splitPath(opt_path[1], shark_interval, max_traj_time)
        return {"path length": opt_path[0], "path": [opt_path[1], path], "cost": opt_cost, "cost list": opt_cost_list}
        

    def planning(self, bin_interval=5, v=1, traj_time_stamp=False, max_plan_time=5, max_traj_time=200.0, plan_time=True):
        """
        rrt path planning
        animation: flag for animation on or off
        """

        self.mps_list = [self.start]
        self.t_start = time.time()
        t_end = time.time() + max_plan_time
        if traj_time_stamp:
            time_expand = math.ceil(max_traj_time / bin_interval)
            for i in range(1, time_expand + 1):
                self.time_bin[bin_interval * i] = []
            self.time_bin[bin_interval].append(self.start)
        while time.time()<t_end:
            #find the closest motion_plan_state by generating a random time stamp and 
            #find the motion_plan_state whose time stamp is closest to it
            if plan_time:
                if traj_time_stamp:
                    ran_bin = int(random.uniform(1, time_expand + 1))
                    while self.time_bin[bin_interval * ran_bin] == []:
                        ran_bin = int(random.uniform(1, time_expand + 1))
                    ran_index = int(random.uniform(0, len(self.time_bin[bin_interval * ran_bin])))
                    closest_mps = self.time_bin[bin_interval * ran_bin][ran_index]
                else:
                    ran_time = random.uniform(0, max_plan_time * self.freq)
                    closest_mps = self.get_closest_mps_time(ran_time, self.mps_list)
                    if closest_mps.traj_time_stamp > max_traj_time:
                        continue
            #find the closest motion_plan_state by generating a random motion_plan_state
            #and find the motion_plan_state with smallest distance
            else:
                ran_mps = self.get_random_mps()
                closest_mps = self.get_closest_mps(ran_mps, self.mps_list)
                if closest_mps.traj_time_stamp > max_traj_time:
                    continue
            
            new_mps = self.steer(closest_mps, self.dist_to_end, self.diff_max, self.freq, v, traj_time_stamp)
            if self.check_collision(new_mps, self.obstacle_list):
                print(new_mps)
                new_mps.parent = closest_mps
                self.mps_list.append(new_mps)
                #add to time stamp bin
                if traj_time_stamp:
                    curr_bin = (new_mps.traj_time_stamp // bin_interval + 1) * bin_interval
                    if curr_bin > max_traj_time:
                        #continue
                        self.time_bin[curr_bin] = []
                    self.time_bin[curr_bin].append(new_mps)
                
            final_mps = self.connect_to_goal_curve_alt(self.mps_list[-1], self.exp_rate)
            if self.check_collision(final_mps, self.obstacle_list):
                final_mps.parent = self.mps_list[-1]
                path = self.generate_final_course(final_mps)
                return path
        
        return None  # cannot find path

    def steer(self, mps, dist_to_end, diff_max, freq, velocity=1, traj_time_stamp=False):
        #dubins library
        '''new_mps = Motion_plan_state(from_mps.x, from_mps.y, theta = from_mps.theta)
        new_mps.path = []
        q0 = (from_mps.x, from_mps.y, from_mps.theta)
        q1 = (to_mps.x, to_mps.y, to_mps.theta)
        turning_radius = 1.0
        path = dubins.shortest_path(q0, q1, turning_radius)
        configurations, _ = path.sample_many(exp_rate)
        for configuration in configurations:
            new_mps.path.append(Motion_plan_state(x = configuration[0], y = configuration[1], theta = configuration[2]))
        new_mps.path.append(to_mps)
        dubins_path = new_mps.path
        new_mps = dubins_path[-2]
        new_mps.path = dubins_path'''

        if traj_time_stamp:
            new_mps = Motion_plan_state(mps.x, mps.y, theta = mps.theta, traj_time_stamp=mps.traj_time_stamp)
        else:
            new_mps = Motion_plan_state(mps.x, mps.y, theta = mps.theta, plan_time_stamp=time.time()-self.t_start, traj_time_stamp=mps.traj_time_stamp)

        new_mps.path = [mps]

        '''if dist_to_end > d:
            dist_to_end = dist_to_end / 10
        n_expand = math.floor(dist_to_end / exp_rate)'''

        n_expand = random.uniform(0, freq)
        n_expand = math.floor(n_expand/1)

        for _ in range(n_expand):
            #setting random parameters
            dist = random.uniform(0, dist_to_end)#setting random range
            diff = random.uniform(-diff_max, diff_max)#setting random range
            velocity = random.uniform(0, 2*velocity)

            if abs(dist) > abs(diff):

                s1 = dist + diff
                s2 = dist - diff
                radius = (s1 + s2)/(-s1 + s2)
                phi = (s1 + s2)/ (2 * radius)
                
                ori_theta = new_mps.theta
                new_mps.theta += phi
                delta_x = radius * (math.sin(new_mps.theta) - math.sin(ori_theta))
                delta_y = radius * (-math.cos(new_mps.theta) + math.cos(ori_theta))
                new_mps.x += delta_x
                new_mps.y += delta_y
                if traj_time_stamp:
                    new_mps.traj_time_stamp += (math.sqrt(delta_x ** 2 + delta_y ** 2)) / velocity
                else:
                    new_mps.plan_time_stamp = time.time() - self.t_start
                    new_mps.traj_time_stamp += (math.sqrt(delta_x ** 2 + delta_y ** 2)) / velocity
                new_mps.path.append(Motion_plan_state(new_mps.x, new_mps.y, theta=new_mps.theta, traj_time_stamp=new_mps.traj_time_stamp, plan_time_stamp=new_mps.plan_time_stamp))

            #d, theta = self.get_distance_angle(new_mps, to_mps)

        '''d, _ = self.get_distance_angle(new_mps, to_mps)
        if d <= dist_to_end:
            new_mps.path.append(to_mps)'''

        #new_node.parent = from_node
        new_mps.path[0] = mps

        return new_mps

    def connect_to_goal(self, mps, exp_rate, dist_to_end=float("inf")):
        new_mps = Motion_plan_state(mps.x, mps.y)
        d, theta = self.get_distance_angle(new_mps, self.goal)

        new_mps.path = [new_mps]

        if dist_to_end > d:
            dist_to_end = d

        n_expand = math.floor(dist_to_end / exp_rate)

        for _ in range(n_expand):
            new_mps.x += exp_rate * math.cos(theta)
            new_mps.y += exp_rate * math.sin(theta)
            new_mps.path.append(Motion_plan_state(new_mps.x, new_mps.y))

        d, _ = self.get_distance_angle(new_mps, self.goal)
        if d <= dist_to_end:
            new_mps.path.append(self.goal)
        
        new_mps.path[0] = mps

        return new_mps
    
    def generate_final_course(self, mps):
        path = [mps]
        mps = mps
        while mps.parent is not None:
            reversed_path = reversed(mps.path)
            for point in reversed_path:
                path.append(point)
            mps = mps.parent
        #path.append(mps)

        return path

    def get_random_mps(self, size_max=15):
        x_max = max([mps.x for mps in self.boundary])
        x_min = min([mps.x for mps in self.boundary])
        y_max = max([mps.y for mps in self.boundary])
        y_min = min([mps.y for mps in self.boundary])

        ran_x = random.uniform(x_min, x_max)
        ran_y = random.uniform(y_min, y_max)
        ran_theta = random.uniform(-math.pi, math.pi)
        ran_size = random.uniform(0, size_max)
        mps = Motion_plan_state(ran_x, ran_y, theta=ran_theta, size=ran_size)
        #ran_z = random.uniform(self.min_area.z, self.max_area.z)
        
        return mps

    def draw_graph(self, traj_path, rnd=None):
        fig = plt.figure(1, figsize=(10,8))
        x,y = self.boundary_poly.exterior.xy
        for i in range(len(list(self.sharkGrid.keys()))):
            ax = fig.add_subplot(2,2,i+1)
            ax.plot(x, y, color="black")
        # for mps in self.mps_list:
        #     if mps.parent:
        #         plt.plot([point.x for point in mps.path], [point.y for point in mps.path], '-g')

            # plot obstacels as circles 
            for obs in self.obstacle_list:
                ax.add_patch(plt.Circle((obs.x, obs.y), obs.size, color = '#000000', fill = False))
        
            for habitat in self.habitats:
                ax.add_patch(plt.Circle((habitat.x, habitat.y), habitat.size, color = 'b', fill = False))
            
            patch = []
            occ = []
            key = list(self.sharkGrid.keys())[i]
            for cell in self.cell_list:
                polygon = patches.Polygon(list(cell.exterior.coords), True)
                patch.append(polygon)
                occ.append(self.sharkGrid[key][cell.bounds])
            
            p = collections.PatchCollection(patch)
            p.set_cmap("Greys")
            p.set_array(np.array(occ))
            ax.add_collection(p)
            fig.colorbar(p, ax=ax)

            ax.set_xlim([self.boundary_poly.bounds[0]-10, self.boundary_poly.bounds[2]+10])
            ax.set_ylim([self.boundary_poly.bounds[1]-10, self.boundary_poly.bounds[3]+10])

            ax.title.set_text(str(list(self.sharkGrid.keys())[i]))
            
            ax.plot([mps.x for mps in traj_path[0]], [mps.y for mps in traj_path[0]], "r")
            ax.plot([mps.x for mps in traj_path[1][key]], [mps.y for mps in traj_path[1][key]], 'b')

        plt.show()

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)
    
    def connect_to_goal_curve(self, mps1):
        a = (self.goal.y - mps1.y)/(np.cosh(self.goal.x) - np.cosh(mps1.x))
        b = mps1.y - a * np.cosh(mps1.x)
        x = np.linspace(mps1.x, self.goal.x, 15)
        x = x.tolist()
        y = a * np.cosh(x) + b
        y = y.tolist()

        new_mps = Motion_plan_state(x[-2], y[-2])
        new_mps.path.append(mps1)
        for i in range(1, len(x)):
            new_mps.path.append(Motion_plan_state(x[i], y[i]))
            new_mps.length += math.sqrt((new_mps.path[i].x-new_mps.path[i-1].x) ** 2 +  (new_mps.path[i].y-new_mps.path[i-1].y) ** 2)
        '''plt.plot(mps1.x, mps1.y, color)
        plt.plot(self.goal.x, self.goal.y, color)
        plt.plot([mps.x for mps in new_mps.path], [mps.y for mps in new_mps.path], color)
        plt.show()'''
        return new_mps
    
    def connect_to_goal_curve_alt(self, mps, exp_rate):
        new_mps = Motion_plan_state(mps.x, mps.y, theta=mps.theta, traj_time_stamp=mps.traj_time_stamp)
        theta_0 = new_mps.theta
        _, theta = self.get_distance_angle(mps, self.goal)
        diff = theta - theta_0
        diff = self.angle_wrap(diff)
        if abs(diff) > math.pi / 2:
            return

        #polar coordinate
        r_G = math.hypot(self.goal.x - new_mps.x, self.goal.y - new_mps.y)
        phi_G = math.atan2(self.goal.y - new_mps.y, self.goal.x - new_mps.x)

        #arc
        phi = 2 * self.angle_wrap(phi_G - new_mps.theta)
        radius = r_G / (2 * math.sin(phi_G - new_mps.theta))

        length = radius * phi
        if phi > math.pi:
            phi -= 2 * math.pi
            length = -radius * phi
        elif phi < -math.pi:
            phi += 2 * math.pi
            length = -radius * phi
        new_mps.length += length

        ang_vel = phi / (length / exp_rate)

        #center of rotation
        x_C = new_mps.x - radius * math.sin(new_mps.theta)
        y_C = new_mps.y + radius * math.cos(new_mps.theta)

        n_expand = math.floor(length / exp_rate)
        for i in range(n_expand+1):
            new_mps.x = x_C + radius * math.sin(ang_vel * i + theta_0)
            new_mps.y = y_C - radius * math.cos(ang_vel * i + theta_0)
            new_mps.theta = ang_vel * i + theta_0
            new_mps.path.append(Motion_plan_state(new_mps.x, new_mps.y, theta = new_mps.theta, plan_time_stamp=time.time()-self.t_start))
        
        return new_mps

    def angle_wrap(self, ang):
        if -math.pi <= ang <= math.pi:
            return ang
        elif ang > math.pi: 
            ang += (-2 * math.pi)
            return self.angle_wrap(ang)
        elif ang < -math.pi: 
            ang += (2 * math.pi)
            return self.angle_wrap(ang)

    def get_closest_mps(self, ran_mps, mps_list):
        min_dist, _ = self.get_distance_angle(mps_list[0],ran_mps)
        closest_mps = mps_list[0]
        for mps in mps_list:
            dist, _ = self.get_distance_angle(mps, ran_mps)
            if dist < min_dist:
                min_dist = dist
                closest_mps = mps
        return closest_mps
    
    def get_closest_mps_time(self, ran_time, mps_list):
        while len(mps_list) > 3:
            left = mps_list[len(mps_list)//2 - 1]
            left_diff = abs(left.plan_time_stamp - ran_time)
            right = mps_list[len(mps_list)//2 + 1]
            right_diff = abs(right.plan_time_stamp - ran_time)

            index =  len(mps_list)//2 
            if left_diff >= right_diff:
                mps_list = mps_list[index:]
            else:
                mps_list = mps_list[:index]
        
        return mps_list[0]

    def check_collision(self, mps, obstacleList):

        if mps is None:
            return False

        dList = []
        for obstacle in obstacleList:
            for point in mps.path:
               d, _ = self.get_distance_angle(obstacle, point)
               dList.append(d) 

            if min(dList) <= obstacle.size:
                return False  # collision
        
        for point in mps.path:
            point = Point(point.x, point.y)
            if not point.within(self.boundary_poly):
                return False

        return True  # safe
    
    def check_collision_obstacle(self, mps, obstacleList):
        for obstacle in obstacleList:
            d, _ = self.get_distance_angle(obstacle, mps)
            if d <= obstacle.size:
                return False
        return True

    def get_distance_angle(self, start_mps, end_mps):
        dx = end_mps.x-start_mps.x
        dy = end_mps.y-start_mps.y
        #dz = end_mps.z-start_mps.z
        dist = math.sqrt(dx**2 + dy**2)
        theta = math.atan2(dy,dx)
        return dist, theta
    
    def cal_length(self, path):
        length = 0
        for i in range(1, len(path)):
            length += math.sqrt((path[i].x-path[i-1].x)**2 + (path[i].y-path[i-1].y)**2)
        return length
    
    def cal_boundary_peri(self):
        peri = 0
        for i in range(len(self.boundary)-1):
            dist, _ = self.get_distance_angle(self.boundary[i], self.boundary[i+1])
            peri += dist
        
        return peri
    
    def plot_performance(self, time_list, perf_list):
        _, ax = plt.subplots()
        ax.plot(time_list, perf_list)
        # Add some text for labels, title and custom x-axis tick labels, etc.
        ax.set_ylabel('optimal sum cost')
        ax.set_title('RRT performance')
        ax.legend()

        plt.show()
    
    def splitPath(self, path, shark_interval, traj_time):
        n_expand = math.floor( traj_time / shark_interval)
        res = {}
        for i in range(n_expand):
            res[(i * shark_interval, (i + 1) * shark_interval)] = []
        
        for point in path:
            for time, arr in res.items():
                if point.traj_time_stamp >= time[0] and point.traj_time_stamp <= time[1]:
                    arr.append(point)
                    break
        return res
    
def createSharkGrid(filepath, cell_list):
    test = {}
    with open(filepath, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:

            string = row["time bin"]
            strings = string.split(", ")
            key = []
            key.append(int(strings[0][1:]))
            key.append(int(strings[1][0:len(strings[1])-1]))

            temp = row["grid"][1:len(row["grid"])-1]
            temp = temp.split(", ")
            val = {}
            for i in range(len(temp)):
                val[cell_list[i].bounds] = float(temp[i])
            test[(key[0],key[1])] = val
    return test

start = catalina.create_cartesian(catalina.START, catalina.ORIGIN_BOUND)
start = Motion_plan_state(start[0], start[1])

goal = catalina.create_cartesian(catalina.GOAL, catalina.ORIGIN_BOUND)
goal = Motion_plan_state(goal[0], goal[1])

obstacles = []
for ob in catalina.OBSTACLES:
    pos = catalina.create_cartesian((ob.x, ob.y), catalina.ORIGIN_BOUND)
    obstacles.append(Motion_plan_state(pos[0], pos[1], size=ob.size))
        
boundary = []
boundary_poly = []
for b in catalina.BOUNDARIES:
    pos = catalina.create_cartesian((b.x, b.y), catalina.ORIGIN_BOUND)
    boundary.append(Motion_plan_state(pos[0], pos[1]))
    boundary_poly.append((pos[0],pos[1]))
boundary_poly = Polygon(boundary_poly)
        
boat_list = []
for boat in catalina.BOATS:
    pos = catalina.create_cartesian((boat.x, boat.y), catalina.ORIGIN_BOUND)
    boat_list.append(Motion_plan_state(pos[0], pos[1], size=boat.size))
        
#testing data for habitats
habitats = []
for habitat in catalina.HABITATS:
    pos = catalina.create_cartesian((habitat.x, habitat.y), catalina.ORIGIN_BOUND)
    habitats.append(Motion_plan_state(pos[0], pos[1], size=habitat.size))
    
# testing data for shark trajectories
shark_dict = shark_dict = {1: [Motion_plan_state(-120 + (0.3 * i), -60 + (0.3 * i), traj_time_stamp=i) for i in range(1,201)], 
2: [Motion_plan_state(-65 - (0.3 * i), -50 + (0.3 * i), traj_time_stamp=i) for i in range(1,201)],
3: [Motion_plan_state(-110 + (0.3 * i), -40 - (0.3 * i), traj_time_stamp=i) for i in range(1,201)], 
4: [Motion_plan_state(-105 - (0.3 * i), -55 + (0.3 * i), traj_time_stamp=i) for i in range(1,201)],
5: [Motion_plan_state(-120 + (0.3 * i), -50 - (0.3 * i), traj_time_stamp=i) for i in range(1,201)], 
6: [Motion_plan_state(-85 - (0.3 * i), -55 + (0.3 * i), traj_time_stamp=i) for i in range(1,201)],
7: [Motion_plan_state(-270 + (0.3 * i), 50 + (0.3 * i), traj_time_stamp=i) for i in range(1,201)], 
8: [Motion_plan_state(-250 - (0.3 * i), 75 + (0.3 * i), traj_time_stamp=i) for i in range(1,201)],
9: [Motion_plan_state(-260 - (0.3 * i), 75 + (0.3 * i), traj_time_stamp=i) for i in range(1,201)], 
10: [Motion_plan_state(-275 + (0.3 * i), 80 - (0.3 * i), traj_time_stamp=i) for i in range(1,201)]}
    
rrt = RRT(Motion_plan_state(-200, 0), start, boundary, obstacles, habitats)
# path = rrt.planning()
path = rrt.exploring(shark_dict, {}, 0.5, 5, 2, 50, traj_time_stamp=True, max_plan_time=20, max_traj_time=200.0, plan_time=True, weights=[1, -3, -1, -5])
print(path["cost"])

# Draw final path
rrt.draw_graph(path["path"])