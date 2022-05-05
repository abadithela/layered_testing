#!/usr/local/bin/python
#
# Lane change simulation

import sys
sys.path.append('..')
import os
from PIL import Image
import matplotlib as mpl
mpl.use('tkagg')
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import time
from ipdb import set_trace as st
from car import Car
import numpy as np
from numpy import cos, sin, tan, sign
from helper import find_corner_coordinates
from parameters import *

def get_background():
    return Image.open(road_fig)

def draw_cars(cars, background):
    for car in all_cars:
        v, theta, x, y = car.state
        theta_d = -theta/np.pi * 180
        vehicle_fig = Image.open(car_figs[car.color])
        w_orig, h_orig = vehicle_fig.size
        vehicle_fig = vehicle_fig.rotate(theta_d, expand = False)
        scaled_vehicle_fig_size  =  tuple([int(car_scale_factor * i) for i in vehicle_fig.size])
        vehicle_fig = vehicle_fig.resize(scaled_vehicle_fig_size)
        x_corner, y_corner = find_corner_coordinates(-car_scale_factor * center_to_axle_dist, 0, x, y, theta, vehicle_fig)
        background.paste(vehicle_fig, (x_corner, y_corner), vehicle_fig)

def update_agents(cars, dt):
    for car in all_cars:
        car.update(car.control_input(), dt)

def animate(frame_idx):
    # st()
    global background
    ax.clear()
    t0 = time.time()
    # update cars
    update_agents(all_cars, dt)
    # draw cars
    draw_cars(all_cars, background)
    # update background
    the_road = [ax.imshow(background, origin="lower")] # update the stage
    background.close()
    background = get_background()

    all_artists = the_road# + the_cars
    t1 = time.time()
    elapsed_time = (t1 - t0)

    return all_artists

show_axes = False
dir_path = os.path.dirname(os.path.realpath(__file__))
road_fig = os.path.dirname(dir_path) + '/layered_testing/imglib/test_road.png'
car_colors = ['red', 'blue']
car_figs = dict()
for color in car_colors:
    car_figs[color] = os.path.dirname(dir_path) + '/layered_testing/imglib/' + color + '_car.png'

# creates figure
fig = plt.figure()
ax = fig.add_axes([0,0,1,1]) # get rid of white border
if not show_axes:
    plt.axis('off')
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)

# sampling time
dt = 0.1
duration = 100
background = get_background()

# list of agents
all_cars = []

system = Car('sys', 0, [5,0,20,69], 'red')
tester = Car('tester', 0, [7,0,20,37], 'blue')
all_cars.append(system)
all_cars.append(tester)

t0 = time.time()
# st()
animate(0)
t1 = time.time()
interval = (t1 - t0)
ani = animation.FuncAnimation(fig, animate, frames=int(duration/dt), interval=interval, blit=True, repeat=False)

plt.show()
t2 = time.time()
print('Total simulation time: ' + str(t2-t0))
