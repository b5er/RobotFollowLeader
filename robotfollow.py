#!/usr/bin/env python3

"""
ENEE322 Robot Follow Project
"""

import argparse
from math import cos, sin, atan2, pi, degrees, sqrt
import matplotlib.pyplot as plt
import random

# UNITS IN METERS, RADIANS, SECONDS, KILOGRAMS

_STAGE_MIN_X = -50
_STAGE_MAX_X =  50
_STAGE_MIN_Y = -50
_STAGE_MAX_Y =  50

_THRESH_ANGLE = .01
_THRESH_DISTANCE = .2

class World:
    def __init__(self, num_robots=1, time_step=.1):
        self.curr_time = 0
        self.time_step = time_step
        self.robots = []
        self.state = None
        self.history = None
        self.running = True
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

    def get_waypoints(self):
        waypoints = []
        for r in self.robots:
            wp = r.get_active_waypoint()
            if wp:
                waypoints.append(wp)
        return waypoints

    def get_robot_histories(self):
        x_hist = []
        y_hist = []
        for r in self.robots:
            x, y = r.get_line()
            x_hist.append(x)
            y_hist.append(y)
        return x_hist, y_hist

    def update(self):
        if not self.running:
            return self.state, self.history

        for r in self.robots:
            r.update_state(self.time_step)
        self.curr_time += self.time_step
        next_state = self.get_robot_states()
        if self.state == next_state:
            self.running = False
        else:
            self.state = next_state
            self.history = self.get_robot_histories()
        return self.state, self.history

class Controller:
    def __init__(self, bots=None):
        self.bots = bots

    def add_bot(self, bot):
        self.bots.append(bot)

    def find_avg_coord(self):
        x = []
        y = []
        for r in self.bots:
            rx, ry, _ = r.get_state()
            x.append(rx)
            y.append(ry)
        avg_x = sum(x)/len(x)
        avg_y = sum(y)/len(y)
        return avg_x, avg_y

    def create_formation(self, formation='line', **kwargs):
        """
        create and assign waypoints to bots based on chosen formation
        for default formation, line, additional kwarg "theta" optional
        """
        if formation == 'line':
            start_x, start_y = self.find_avg_coord()
            if 'theta' in kwargs:
                theta = kwargs['theta']
            else:
                theta = random.uniform(0,pi)
            if 'spacing' in kwargs:
                spacing = kwargs['spacing']
            else:
                spacing = 8
            if 'travel_distance' in kwargs:
                travel_distance = kwargs['travel_distance']
            else:
                travel_distance = 20

            # create 'meetup' and final waypoints, assign them
            meetup_x = start_x
            meetup_y = start_y
            final_waypoint = []
            for r in self.bots:
                r.waypoint = [] #clear any residue waypoints
                ct = cos(theta)
                st = sin(theta)
                r.waypoint.append({'x':meetup_x,'y':meetup_y,'theta':theta})
                final_waypoint.append({'x':meetup_x + travel_distance * ct,
                                     'y':meetup_y + travel_distance * st,
                                     'theta':theta})
                meetup_x -= spacing * ct
                meetup_y -= spacing * st

            def update_func():
                for i, r in enumerate(self.bots):
                    r.waypoint.append(final_waypoint[i])
                
            return update_func # call for next stage of formation

class Robot:
    def update_state(self, dt):
        """ change state based on a timestep and current state"""
        self.act_on_waypoint() #closed loop logic + feedback
        self.perform_open_loop()
        self.physics(dt)
        self.log_position_history()
   
    def physics(self, dt):
        self.limit_speed()
        self.x += 0.5 * self.radius * (self.u_r + self.u_l) * cos(self.theta) * dt
        self.y += 0.5 * self.radius * (self.u_r + self.u_l) * sin(self.theta) * dt
        self.theta += (self.radius / self.length) * (self.u_r - self.u_l) * dt
        self.theta = self.theta % (2*pi)

    def log_position_history(self):
        self.x_history.append(self.x)
        self.y_history.append(self.y)
        self.t_history.append(self.theta)

    def print_state(self):
        print("Robot #" + str(self.idn) + ":")
        print("\tX: " + str(self.x))
        print("\tY: " + str(self.y))
        print("\tθ: " + str(self.theta))
    
    def get_state(self):
        return self.x, self.y, self.theta

    def get_line(self):
        return self.x_history, self.y_history

    def get_history(self):
        return self.x_history, self.y_history, self.t_history

    def set_speed(self, u_r, u_l=None):
        #   one arg: both same speed
        self.u_r = u_r
        if u_l:
            self.u_l = u_l
        else:
            self.u_l = u_r

    def perform_open_loop(self): 
        if self.open_loop:
            self.u_r = self.open_loop_r(self.u_r)
            self.u_l = self.open_loop_l(self.u_l)
    
    def limit_speed(self):
        if self.u_r > self.max_spin:
            self.u_r = self.max_spin
        elif self.u_r < -self.max_spin:
            self.u_r = -self.max_spin
        if self.u_l > self.max_spin:
            self.u_l = self.max_spin
        elif self.u_l < -self.max_spin:
            self.u_l = -self.max_spin
            
    def set_open_loop_control(self, func_right, func_left=None, disable_waypoint=True):
        self.open_loop_r = func_right
        if func_left:
            self.open_loop_l = func_left
        else:
            self.open_loop_l = func_right
        self.open_loop = True
        if disable_waypoint:
            self.enable_waypoint = False

    def get_distance_components(self, x, y):
        dx = x - self.x
        dy = y - self.y
        #causes errors due to inaccurate theta
        """
        if abs(dx) < _THRESH_DISTANCE:
            dx = 0
        if abs(dy) < _THRESH_DISTANCE:
            dy = 0
        """
        if sqrt(dx**2 + dy**2) < _THRESH_DISTANCE:
            dx = 0
            dy = 0
        return dx, dy

    def get_desired_theta(self, dx, dy):
        return atan2(dy,dx) % (2*pi)

    def turn(self, d_theta):
        delta_theta = self.theta - d_theta
        speed = self.tk * abs(delta_theta)
        if (delta_theta % 6.28) < 3.14:
            #turn cw
            speed *= -1
        #else:
            #turn ccw
        self.set_speed(speed, -speed)

    def forward(self, distance):
        speed = self.dk * distance
        self.set_speed(speed)

    def act_on_waypoint(self):
        if not self.enable_waypoint:
            return
        if not self.waypoint:
            # no more waypoints
            self.set_speed(0, 0)
            return 
        dx, dy = self.get_distance_components(self.waypoint[0]['x'], self.waypoint[0]['y'])
        arrived = (dx == 0 and dy == 0)
        if arrived:
            self.waypoint[0]['x'] = self.x
            self.waypoint[0]['y'] = self.y

            # theta is allowed to be None. if so, keep robot at same heading
            d_theta = self.waypoint[0]['theta']
            if not d_theta:
                d_theta = self.theta
        else:
            d_theta = self.get_desired_theta(dx, dy)
        #delta theta = theta - desired theta
        dtheta = self.theta - d_theta

        #if we need to turn, turn
        if abs(dtheta) > _THRESH_ANGLE:
            self.turn(d_theta)
        #otherwise, if we need to drive, drive
        elif not arrived:
            self.forward(sqrt(dx**2 + dy**2))
        #otherwise, we're at the current waypoint and can move to next
        else:
            self.waypoint.pop(0)
            self.act_on_waypoint()
    
    def get_active_waypoint(self):
        if self.enable_waypoint and self.waypoint:
            return self.waypoint[0]
        return None
    
    def __init__(self, idn, radius=4, length=10, x=0, y=0, theta=0, max_spin=1):
        self.idn = idn
        self.radius = radius
        self.length = length
        self.x = x
        self.y = y
        self.theta = theta
        self.u_r = 0
        self.u_l = 0
        self.max_spin = max_spin
        self.open_loop = False
        self.enable_waypoint = True
        self.waypoint = []

        # closed loop gains, turning/driving
        self.tk = 0.9
        self.dk = 0.9
        
        #used to graph line where bot has traveled
        self.x_history = [x]
        self.y_history = [y]
        self.t_history = [theta]

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

    # graph robot waypoints
    waypoints = plot_waypoints(world)
    
    # graph lines of robot history
    x, y = lines
    line = []
    for r in range(len(x)):
        line.append(plt.plot(x[r], y[r], color='b', zorder=1))

    plt.pause(.001) #its probably pausing for longer than 1ms
    if clear:
        for bd in bot_dot:
            bd.remove()
        for wp in waypoints:
            wp.remove()
        for l in line:
            lp = l.pop(0)
            lp.remove()

def plot_waypoints(world):
    waypoints = world.get_waypoints()
    wp = []
    for w in waypoints:
        wp.append(plt.scatter(w['x'], w['y'], color='r', zorder=4, marker='x', s=200))
    return wp

def simulate(world, title='Bots', steps=None, infinite=False, next_func=None):
    def run():
        while world.running or infinite:
            world.update()
            draw(world)
        if next_func:
            next_func.pop(0)() #call next function in list, continue simulation
            world.running = True
            run()

    create_plot(title)
    #if limited time set
    if steps:
        for i in range(steps):
            world.update()
            draw(world)
    #otherwise simulate until finished
    else:
        run()  
            
    # display one more frame for a bit, then close
    draw(world, clear=False)
    plt.pause(2) 
    plt.close()

def gen_waypoint_list(x,y,th):
    return [{'x':x[w],'y':y[w],'theta':th[w]} for w in range(len(x))]

def convert_tpt_to_waypoints(tpt):
    """
    simplify target pose trajectory for use by bots
    assumes turning and translating are exclusive
    """
    if not tpt: #empty 
        return 
   
    def theta_different(t1, t2):
        return abs(t1 - t2) > _THRESH_ANGLE

    def distance_different(d1, d2):
        return abs(d1 - d2) > _THRESH_DISTANCE

    all_waypoints = tpt 
    turning = False
    driving = False
    prev_w = all_waypoints[0]
    waypoints = []
    waypoints.append(all_waypoints[0])
    for curr_w in all_waypoints:
        if theta_different(curr_w['theta'], prev_w['theta']):
            turning = True
        elif turning:
            waypoints.append(curr_w)
            turning = False
        if distance_different(curr_w['x'], prev_w['x']) or distance_different(curr_w['y'], prev_w['y']):
            driving = True
        elif driving:
            waypoints.append(curr_w)
            driving = False
        prev_w = curr_w
    waypoints.append(all_waypoints[-1])
    return waypoints

def create_tpt_data(num_waypoints=4):
    def rand_pose():
        x = random.uniform(_STAGE_MIN_X, _STAGE_MAX_X)
        y = random.uniform(_STAGE_MIN_Y, _STAGE_MAX_Y)
        theta = random.uniform(0, pi)
        return {'x':x, 'y':y, 'theta':theta}
    print("Creating Target Pose Trajectory:")
    waypoints = []
    for i in range(num_waypoints):
        waypoints.append(rand_pose())
    init_pose = waypoints[0]
    rob = Robot(0, x=init_pose['x'], y=init_pose['y'], theta=init_pose['theta'], max_spin=.01)
    rob.waypoint = waypoints
    prev_state = rob.get_state()
    curr_state = None
    prev_wp_remaining = len(rob.waypoint)
    curr_wp_remaining = None
    while curr_state != prev_state:
        #progress
        curr_wp_remaining = len(rob.waypoint)
        if curr_wp_remaining != prev_wp_remaining:
            print("\t" + str(curr_wp_remaining) + " Waypoints Remaining")
            prev_wp_remaining = curr_wp_remaining
        #step physics
        rob.update_state(.1)
        prev_state = curr_state
        curr_state = rob.get_state()
    print("Done")
    return gen_waypoint_list(*rob.get_history())
    
def problem1():
    # create single robot with open loop braking
    rob = Robot(0, theta=pi/3, max_spin=10)
    rob.set_speed(10)
    rob.set_open_loop_control(lambda x: .9 * x, disable_waypoint=True)

    # create world, add robot
    world = World(0)
    world.add_bot(rob)

    simulate(world, 'Problem 1: Open Loop Control (Braking)', 10)

    world.clear_bots()
    rob = Robot(0, theta=pi/3, max_spin=10)
    rob.set_speed(1,-1)
    world.add_bot(rob)
    rob.set_open_loop_control(lambda x: 1.01 * x, disable_waypoint=True)
    simulate(world, 'Problem 1: Open Loop Control (Rotating, Accelerating)', 70)

    world.clear_bots()
    rob = Robot(0, theta=pi/3, max_spin=10)
    rob.set_speed(1, 2.2)
    rob.set_open_loop_control(lambda right: 1.01 * right, lambda left: 1.008 * left, disable_waypoint=True)
    world.add_bot(rob)
    simulate(world, 'Problem 1: Open Loop Control (Turning, Accelerating)', 120)

def problem2():
    world = World(0)
    rob = Robot(0)
    rob.waypoint = [{'x':20,'y':30,'theta':pi/2}]
    world.add_bot(rob)
    simulate(world, 'Problem 2: Closed Loop Control')

    world.clear_bots()
    world = World(0)
    bots = []
    r0 = Robot(0,x=-35,y=30)
    r0.waypoint = gen_waypoint_list(
        [-30,-25,-20,-15,-10,-5,0],
        [-20,16,-10,8,-4,0,0],
        [None]*7
    )
    bots.append(r0)
    r1 = Robot(1,x=-10,y=40)
    r1.waypoint = gen_waypoint_list(
        [-5,20],
        [10,14],
        [None]*3
    )
    bots.append(r1)
    r2 = Robot(2,x=10,y=10)
    r2.waypoint = gen_waypoint_list(
        [40,20],
        [30,20],
        [None]*2
    )
    bots.append(r2)
    r3 = Robot(3,x=18,y=-38)
    r3.waypoint = gen_waypoint_list(
        [25,38,6,18],
        [-34,18,-5,-38],
        [None]*4 
    )
    bots.append(r3)
    r4 = Robot(3,x=38,y=18)
    r4.waypoint = gen_waypoint_list(
        [6,18,25,38],
        [-5,-38,-34,18],
        [None]*4 
    )
    bots.append(r4)
    for r in bots:
        world.add_bot(r)
    simulate(world, 'Problem 2: Closed Loop Control (Multiple Robots)')

def problem3():
    world = World(0)
    rob = Robot(0)
    tpt = create_tpt_data(5)
    wp = convert_tpt_to_waypoints(tpt)
    rob.waypoint = wp
    world.add_bot(rob)
    simulate(world, 'Problem 3: Intercepting Target Pose Trajectory')

def problem4():
    world = World(0)
    rob1 = Robot(0, x=30, y=20)
    rob2 = Robot(1, x=-40, y=-40)
    world.add_bot(rob1)
    world.add_bot(rob2)
    ctl = Controller(world.robots)
    line_proceed = ctl.create_formation(theta=0.4)
    simulate(world, 'Problem 4: 2 Robots in Line Formation', next_func=[line_proceed])

def problem5(n=8):
    world = World(n)
    ctl = Controller(world.robots)
    line_proceed = ctl.create_formation() 
    simulate(world, 'Problem 5: N robots in Line Formation (N=' + str(n) + ')', next_func=[line_proceed])

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
    problem3()
    problem4()
    problem5()

if __name__ == '__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument('prob_num', type=int, nargs='?')
    parser.add_argument('-n', type=int)
    args = parser.parse_args()
    if args.n:
        problem5(n=args.n)
    elif args.prob_num:
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
