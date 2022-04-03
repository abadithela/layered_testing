## CARLO implementation of a car interacting with a hesitating pedestrian.
## Apurva Badithela
## 3/4/22

import numpy as np
import os
import sys
sys.path.append("../carlo")
from world import World
from agents import Car, CircleBuilding, RingBuilding, RectangleBuilding, Pedestrian, Painting
from geometry import Point
import time
from tkinter import *
import pdb
from bayes_opt import BayesianOptimization

dt=0.1
num_lanes = 2
lane_marker_width = 0.5
num_of_lane_markers = 50
lane_width = 3.5

def setup_track():
    # Setting up circular track:
    dt = 0.1 # time steps in terms of seconds. In other words, 1/dt is the FPS.
    world_width = 120 # in meters
    world_height = 120
    inner_building_radius = 30
    w = World(dt, width = world_width, height = world_height, ppm = 6) # The world is 120 meters by 120 meters. ppm is the pixels per meter.

    # Road setup
    # Circular crossing
    # To create a circular road, we will add a CircleBuilding and then a RingBuilding around it
    cb = CircleBuilding(Point(world_width/2, world_height/2), inner_building_radius, 'gray80')
    w.add(cb)
    rb = RingBuilding(Point(world_width/2, world_height/2), inner_building_radius + num_lanes * lane_width + (num_lanes - 1) * lane_marker_width, 1+np.sqrt((world_width/2)**2 + (world_height/2)**2), 'gray80')
    w.add(rb)

    ## Add lane markers:
    # Let's also add some lane markers on the ground. This is just decorative. Because, why not.
    for lane_no in range(num_lanes - 1):
        lane_markers_radius = inner_building_radius + (lane_no + 1) * lane_width + (lane_no + 0.5) * lane_marker_width
        lane_marker_height = np.sqrt(2*(lane_markers_radius**2)*(1-np.cos((2*np.pi)/(2*num_of_lane_markers)))) # approximate the circle with a polygon and then use cosine theorem
        for theta in np.arange(0, 2*np.pi, 2*np.pi / num_of_lane_markers):
            dx = lane_markers_radius * np.cos(theta)
            dy = lane_markers_radius * np.sin(theta)
            w.add(Painting(Point(world_width/2 + dx, world_height/2 + dy), Point(lane_marker_width, lane_marker_height), 'white', heading = theta))

    ## Add zebra crossing:
    w.add(Painting(Point(60, 97), Point(2, 0.5), 'white'))
    w.add(Painting(Point(60, 96), Point(2, 0.5), 'white'))
    w.add(Painting(Point(60, 95), Point(2, 0.5), 'white'))
    w.add(Painting(Point(60, 94), Point(2, 0.5), 'white'))
    w.add(Painting(Point(60, 93), Point(2, 0.5), 'white'))
    w.add(Painting(Point(60, 92), Point(2, 0.5), 'white'))
    w.add(Painting(Point(60, 91), Point(2, 0.5), 'white'))
    w.add(Painting(Point(60, 90), Point(2, 0.5), 'white'))

    return w, cb, rb # Return world

# Setup car object:
def init_car(w, vx=10.0, vy=10.0, max_speed=30.0):
    ## Dynamic objects:
    # System under test (Car)
    # A Car object is a dynamic object -- it can move. We construct it using its center location and heading angle.
    c1 = Car(Point(25.0,60), np.pi/2)
    c1.max_speed = max_speed # let's say the maximum is 30 m/s (108 km/h)
    c1.velocity = Point(vx, vy)
    w.add(c1)
    return c1

# Setup pedestrian object:
def init_ped(w, max_speed=10.0):
    ## Pedestrians:
    # Test environment
    # Pedestrian is almost the same as Car. It is a "circle" object rather than a rectangle.
    p1 = Pedestrian(Point(60,98), 3*np.pi/2)
    p1.max_speed = max_speed # We can specify min_speed and max_speed of a Pedestrian (and of a Car). This is 10 m/s, almost Usain Bolt.
    w.add(p1)
    return p1

# Initial control:
def init_control(p1, c1, p1_st=0, p1_acc = 0.22, c1_st=-0.1, c1_acc = 0.):
    # Visualize pedestrian walking on cross walk:
    # Let's implement some simple scenario with all agents
    p1.set_control(p1_st, p1_acc) # The pedestrian will have 0 steering and 0.22 throttle. So it will not change its direction.
    c1.set_control(c1_st, c1_acc)

# Def: evaluate robustness:#
# in this case, the constraint is safety, so we take the minimum of the generate_trajectory
#TODO: interface with breach to check robustness of arbitrary STL specifications
def evaluate_robustness(trajectory):
    trajectory_arr = np.array(trajectory)
    return trajectory_arr.min(), trajectory_arr.argmin()

## Visualize car:
# Let's implement some simple policy for the car c1
def generate_trajectory(w, c1, cb, rb, p1):
    desired_lane = 1
    trajectory = [c1.distanceTo(p1)] # Minimum distance between pedestrian and car\

    for k in range(300):
        lp = 0.
        if c1.distanceTo(cb) < desired_lane*(lane_width + lane_marker_width) + 0.2:
            lp += 0.
        elif c1.distanceTo(rb) < (num_lanes - desired_lane - 1)*(lane_width + lane_marker_width) + 0.3:
            lp += 1.

        v = cb.center - c1.center
        v = np.mod(np.arctan2(v.y, v.x) + np.pi/2, 2*np.pi)
        if c1.heading < v:
            lp += 0.4
        else:
            lp += 0.

        if np.random.rand() < lp:
            c1.set_control(-0.2, 0.)
        else:
            c1.set_control(-0.1, 0)

        w.tick() # This ticks the world for one time step (dt second)
        trajectory.append(c1.distanceTo(p1))
        time.sleep(dt/4) # Let's watch it 4x

        # if w.collision_exists(p1): # We can check if the Pedestrian is currently involved in a collision. We could also check c1 or c2.
        #     print('Pedestrian has died!')
        # if w.collision_exists(): # We can check if there is any collision at all.
        #     print('Collision exists somewhere...')
    w.close()
    return trajectory

## Visualize car:
# Let's implement some simple policy for the car c1
def plot_trajectory(w, c1, cb, rb, p1):
    desired_lane = 1
    trajectory = [c1.distanceTo(p1)] # Minimum distance between pedestrian and car\

    for k in range(300):
        lp = 0.
        if c1.distanceTo(cb) < desired_lane*(lane_width + lane_marker_width) + 0.2:
            lp += 0.
        elif c1.distanceTo(rb) < (num_lanes - desired_lane - 1)*(lane_width + lane_marker_width) + 0.3:
            lp += 1.

        v = cb.center - c1.center
        v = np.mod(np.arctan2(v.y, v.x) + np.pi/2, 2*np.pi)
        if c1.heading < v:
            lp += 0.4
        else:
            lp += 0.

        if np.random.rand() < lp:
            c1.set_control(-0.2, 0.)
        else:
            c1.set_control(-0.1, 0)

        w.tick() # This ticks the world for one time step (dt second)
        trajectory.append(c1.distanceTo(p1))
        w.render()
        time.sleep(dt/4) # Let's watch it 4x

        if w.collision_exists(p1): # We can check if the Pedestrian is currently involved in a collision. We could also check c1 or c2.
            print('Pedestrian has died!')
        if w.collision_exists(): # We can check if there is any collision at all.
            print('Collision exists somewhere...')
    w.close()
    return trajectory

# Black-box function to be optimized/maximized:
# Test parameters: initial speed of the car (vx, vy), acceleration of pedestrian (p1_acc) and steering angle of car (c1_st)
def black_box_function(vx, vy, p1_acc, c1_st):
    w, cb, rb = setup_track()
    c1 = init_car(w, vx=vx, vy=vy, max_speed=30.0)
    p1 = init_ped(w, max_speed=10.0)
    init_control(p1, c1, p1_st=0, p1_acc = p1_acc, c1_st=c1_st, c1_acc = 0.)
    trajectory = generate_trajectory(w, c1, cb, rb, p1)
    rho, argmin_rho = evaluate_robustness(trajectory)
    return -rho

# Plots a specific trajectory
def run_traj_inputs(vx, vy, p1_acc, c1_st):
    w, cb, rb = setup_track()
    c1 = init_car(w, vx=vx, vy=vy, max_speed=30.0)
    p1 = init_ped(w, max_speed=10.0)
    init_control(p1, c1, p1_st=0, p1_acc = p1_acc, c1_st=c1_st, c1_acc = 0.)
    trajectory = plot_trajectory(w, c1, cb, rb, p1)

# Find worst-case inputs (initial conditions) to the black-box function:
def find_worst_case_u(pbounds):
    init_points = 5
    n_iters = 10
    optimizer = BayesianOptimization(
        f=black_box_function,
        pbounds=pbounds,
        random_state=1,
    )
    # pdb.set_trace()
    optimizer.maximize(init_points=init_points, n_iter=n_iters)
    return optimizer


if __name__ == '__main__':
    case = 2 # bayes # OPTIMIZE:
    if case == 1:
        w, cb, rb = setup_track()
        c1 = init_car(w, vx=10.0, vy=10.0, max_speed=30.0)
        p1 = init_ped(w, max_speed=10.0)
        init_control(p1, c1, p1_st=0, p1_acc = 0.22, c1_st=-0.1, c1_acc = 0.)
        trajectory = generate_trajectory(w, c1, cb, rb, p1)
        rho, argmin_rho = evaluate_robustness(trajectory)

    elif case == 2:
        pbounds = {'vx': (0.0, 30.0), 'vy': (0.0, 30.0), 'p1_acc': (-0.5, 0.5), 'c1_st': (-0.3, -0.0)}
        optimizer = find_worst_case_u(pbounds)
        optimal_inputs = optimizer.max # Gathering optimal inputs from Bayesian Optimization
        # Plot worst-case trajectory of optimal inputs:
        opt_vx = optimal_inputs['params']['vx']
        opt_vy = optimal_inputs['params']['vy']
        opt_p1_acc = optimal_inputs['params']['p1_acc']
        opt_c1_st = optimal_inputs['params']['c1_st']
        run_traj_inputs(opt_vx, opt_vy, opt_p1_acc, opt_c1_st)
