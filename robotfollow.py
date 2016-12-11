#!/usr/bin/env python3

"""
ENEE322 Robot Follow Project
"""

from math import cos, sin, pi, degrees
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random

_RUN_TIME = 10000 
_STAGE_MIN_X = -50
_STAGE_MAX_X =  50
_STAGE_MIN_Y = -50
_STAGE_MAX_Y =  50

class World:
    def __init__(self, num_robots=1, time_step=.1):
        self.curr_time = 0
        self.time_step = time_step
        self.robots = []
        for i in range(num_robots):
            x = random.randint(_STAGE_MIN_X, _STAGE_MAX_X)
            y = random.randint(_STAGE_MIN_Y, _STAGE_MAX_Y)
            theta = random.uniform(0,pi)
            self.robots.append(Robot(i, x=x, y=y, theta=theta))

    def get_robot_states(self):
        x_list = []
        y_list= []
        theta_list = []
        for r in self.robots:
            x, y, theta = r.get_state()
            x_list.append(x)
            y_list.append(y)
            theta_list.append(theta)
        return x_list, y_list, theta_list

    def get_robot_histories(self):
        x_hist = []
        y_hist = []
        for r in self.robots:
            x, y = r.get_line()
            x_hist.append(x)
            y_hist.append(y)
        return x_hist, y_hist

    def update(self):
        for r in self.robots:
            r.update_state(self.time_step)
        self.curr_time += self.time_step
        return self.get_robot_states(), self.get_robot_histories()

class Robot:
    def update_state(self, dt):
        """ change state based on a timestep and current state"""
        self.x += 0.5 * self.radius * (self.u_r + self.u_l) * cos(self.theta) * dt
        self.y += 0.5 * self.radius * (self.u_r + self.u_l) * sin(self.theta) * dt
        self.theta += (self.radius / self.length) * (self.u_r - self.u_l) * dt
        self.x_history.append(self.x)
        self.y_history.append(self.y)

    def print_state(self):
        print("Robot #" + str(self.idn) + ":")
        print("\tX: " + str(self.x))
        print("\tY: " + str(self.y))
        print("\tθ: " + str(self.theta))
    
    def get_state(self):
        return self.x, self.y, self.theta

    def get_line(self):
        return self.x_history, self.y_history

    def __init__(self, idn, radius=4, length=10, x=0, y=0, theta=0, max_speed=1):
        self.idn = idn
        self.radius = radius
        self.length = length
        self.x = x
        self.y = y
        self.theta = theta
        self.u_r = 0
        self.u_l = 0
        self.max_speed = max_speed
        
        #used to graph line where bot has traveled
        self.x_history = [x]
        self.y_history = [y]

def animate():
    """
    super inefficient method because scatter() doesn't accept array of unique markers
    """
    poses, lines = world.update()

    # plot robot triangles
    x, y, th = poses
    marker = [(3,0,degrees(t)-90) for t in th]
    bot_dot = []
    for m in range(len(x)):
        bot_dot.append(plt.scatter(x[m], y[m], marker=marker[m], s=800, c='g', zorder=2))
    
    # graph lines of robot history
    x, y = lines
    line = []
    for r in range(len(x)):
        line.append(plt.plot(x[r], y[r], color='b', zorder=1))

    plt.pause(.001) #its probably pausing for longer than 1ms
    for bd in bot_dot:
        bd.remove()
    for l in line:
        lp = l.pop(0)
        lp.remove()
 
if __name__ == '__main__':
    #demo world with 8 robots moving in straight lines
    world = World(8)
    for r in world.robots:
        speed = random.random()*3 + .8
        r.u_r = speed
        r.u_l = speed

    # set up figure
    plt.xlim(_STAGE_MIN_X, _STAGE_MAX_X)
    plt.ylim(_STAGE_MIN_Y, _STAGE_MAX_Y)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Robot Follow')
    plt.ion()
    
    # run/animate
    for t in range(_RUN_TIME):
        animate()
