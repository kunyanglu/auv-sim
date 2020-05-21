# add all the stuff we gotta import
import math
import time
import numpy as np
import random
from numpy import random
import matplotlib.pyplot as plt
from numpy.random import uniform 
from numpy.random import randn
from apscheduler.scheduler import Scheduler
import threading

class particleFilter:
    # 2 sets of initial data- shark's initial position and velocity, and position of AUV 
    # output- estimates the sharks position and velocity
    def __init__(self, init_x_shark, init_y_shark, init_theta, init_x_auv, init_y_auv):
        "how you create an object out of the particle Filter class i.e. "
        self.x_shark = init_x_shark
        self.y_shark = init_y_shark
        self.theta = init_theta
        self.x_auv = init_x_auv
        self.y_auv = init_y_auv
    
    def createParticles(self):
        L = 150
        N = 2

        list_of_particles = []
        #generate random x, y such that it follows circle formula
        coordinate_of_particle = []
        x_p = 0 
        y_p = 0 
        v_p = 0 
        theta_p = 0
        count = 0
        dict = {}
        while count <= N-1:
            x_p = random.uniform(-L, L)
            y_p= random.uniform(-L, L)
            weight_p = (1/N)
            if math.sqrt(x_p**2 + y_p**2) <= L:
                count+= 1
                v_p = random.uniform(0,5)
                theta_p = random.uniform(-math.pi, math.pi) 
                coordinate_of_particle.append([x_p, y_p, v_p, theta_p, weight_p])
        #print("particle list")
        #print(coordinate_of_particle)
        for x in range(count):
            for y in coordinate_of_particle:
                dict[x] = y
        #print("dict")
        #print(dict)
        return coordinate_of_particle
        #returns a dictionary w count and the particle coordinates and its weight
    
    def predict(self, dt = 1):

        s_list = self.createParticles()
        sigma_v = 0.3 
        sigma_0 = math.pi/2
        # can consider ignoring this
        print(s_list)
        while True:
            time.sleep(10.0)
            for p in s_list:
                #change the v of particle and account for noise
                p[2] = int(p[2]) + random.uniform(0, sigma_v)
                p[3] = int(p[3]) + random.uniform(0, sigma_0)
                #print("new v, theta")
                #print(p[2], p[3])
                #print("initial ")
                #print(p[0])
                p[0] += p[2] * math.cos(p[3]) * dt 
                #print(p[0])
                p[1] += p[2] * math.sin(p[3]) * dt
                print(p)
        #creating new x and y coordinates based on new v and theta values
    """
    def predict_time(self):
        timeinterval = 10 
        threading.Timer(interval, startTimer).start()
        predict()
    """
        

        

def main():
        test_particle = particleFilter(10, 10 , 30, 20 ,20)
        test_particle.predict()













    