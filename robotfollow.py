#!/usr/bin/env python3

"""
ENEE322 Robot Follow Project
"""

import argparse
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

    def add_bot(self, bot):
        self.robots.append(bot)

    def clear_bots(self):
        self.robots = []

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
        if self.open_loop:
            self.u_r = self.open_loop_r(self.u_r)
            self.u_l = self.open_loop_l(self.u_l)
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

    def set_speed(self, u_r, u_l=None):
        #   one arg: both same speed
        self.u_r = u_r
        if u_l:
            self.u_l = u_l
        else:
            self.u_l = u_r
            
    def set_open_loop_control(self, func_right, func_left=None):
        self.open_loop_r = func_right
        if func_left:
            self.open_loop_l = func_left
        else:
            self.open_loop_l = func_right
        self.open_loop = True

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
        self.open_loop = False
        
        #used to graph line where bot has traveled
        self.x_history = [x]
        self.y_history = [y]

def draw(world, clear=True):
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
    if clear:
        for bd in bot_dot:
            bd.remove()
        for l in line:
            lp = l.pop(0)
            lp.remove()

def simulate(world, steps):
    for i in range(steps):
        world.update()
        draw(world)
    # display one more frame for a second, then close
    draw(world, clear=False)
    plt.pause(1) 
    plt.close()

def problem1():
    create_plot('Problem 1: Open Loop Control (Braking)')
    
    # create single robot with open loop braking
    rob = Robot(0, theta=pi/3)
    rob.set_speed(10)
    rob.set_open_loop_control(lambda x: .9 * x)

    # create world, add robot
    world = World(0)
    world.add_bot(rob)

    simulate(world, 10)

    world.clear_bots()
    create_plot('Problem 1: Open Loop Control (Rotating, Accelerating)')
    rob = Robot(0, theta=pi/3)
    rob.set_speed(1,-1)
    world.add_bot(rob)
    rob.set_open_loop_control(lambda x: 1.01 * x)
    simulate(world, 70)

    world.clear_bots()
    create_plot('Problem 1: Open Loop Control (Turning, Accelerating)')
    rob = Robot(0, theta=pi/3)
    rob.set_speed(1, 2.2)
    rob.set_open_loop_control(lambda right: 1.01 * right, lambda left: 1.008 * left)
    world.add_bot(rob)
    simulate(world, 120)

def problem2():
    create_plot('Problem 2: Closed Loop Control')
    plt.pause(10)

def problem3():
    pass

def problem4():
    pass

def problem5():
    pass

def create_plot(title):
    plt.xlim(_STAGE_MIN_X, _STAGE_MAX_X)
    plt.ylim(_STAGE_MIN_Y, _STAGE_MAX_Y)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title(title)
    plt.ion()

def main():
    problem1()
    problem2()

if __name__ == '__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument('prob_num', type=int, nargs='?')
    args = parser.parse_args()
    if args.prob_num:
        probs = {
            1:problem1, 
            2:problem2, 
            3:problem3, 
            4:problem4,
            5:problem5,
        }
        probs[args.prob_num]()
    else:
        main()
