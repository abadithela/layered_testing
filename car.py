import os
import sys
sys.path.append("..")
import scipy.io
import numpy as np
from scipy.integrate import odeint
from numpy import cos, sin, tan, sign

def in_limits(val, val_min, val_max):
    return max(min(val, val_max), val_min)

class Car():
    def __init__(self,
                 name = None,
                 id = None,
                 init_state=[0, 0, 0, 0],
                 color = 'blue',
                 length = 50,  # length of vehicle in pixels
                 acc_max = 9.81,  # maximum acceleration of vehicle
                 acc_min = -9.81,  # maximum deceleration of vehicle
                 steer_max = 0.5,  # maximum steering input in radians
                 steer_min = -0.5,  # minimum steering input in radians
                 v_max = 100):  # maximum velocity
        self._length = length
        self._v_max = v_max
        self.id = id
        self.destination = None
        self.acc_range = (acc_min, acc_max)
        self.steer_range = (steer_min, steer_max)
        self.state = np.array(init_state, dtype='float')
        self.input = None
        self.color = color

    def update(self, input, dt):
        in_acc = input[0]
        in_steer = input[1]
        # update the state of the car depending on the control inputs for duration dt
        self.state = odeint(self.dynamics, self.state, t=(0, dt), args=(in_acc, in_steer))[1]

    def control_input(self):
        # add the controller here! [acc, steering]
        control_input = [0,0]
        return control_input

    def dynamics(self, state, time, acc, steer):
        # if at max speed, cannot go faster
        if abs(state[0]) >= self._v_max and sign(acc) == sign(state[0]):
            v_dot = 0
        else:
            v_dot = in_limits(acc, self.acc_range[0], self.acc_range[1])
        theta_dot = state[0] / self._length * tan(in_limits(steer, self.steer_range[0], self.steer_range[1]))
        x_dot = state[0] * cos(state[1])
        y_dot = state[0] * sin(state[1])
        state_dot = [v_dot, theta_dot, x_dot, y_dot]
        return state_dot
