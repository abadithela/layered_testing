import os
import sys
sys.path.append("..")
import scipy.io
import numpy as np
from scipy.integrate import odeint
from numpy import cos, sin, tan, sign
from controller import Controller
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
        self.xhist = [self.init_state]
        self.destination = None
        self.acc_range = (acc_min, acc_max)
        self.steer_range = (steer_min, steer_max)
        self.state = np.array(init_state, dtype='float')
        self.input = None
        self.color = color
        self.dt = 0.1

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

    def update_dubins(self, not_merged, can_merge):
        # update the state of the car depending on the control inputs for duration dt
        self.control_mpc(not_merged=not_merged, can_merge =can_merge)

    # Helper functions to generate next waypoints:
    def get_merged_state(self):
        xG = np.array([self.state[0], self.state[1], self.state[2]+1, self.state[3]+1]) # Maintain heading
        return xG

    def get_continue_straight(self):
        xG = np.array([self.state[0], self.state[1], self.state[2]+1, self.state[3]])
        return xG

    def control_mpc(self, not_merged, can_merge):
        # add the controller here! [acc, steering]
        x0 = self.state
        if not_merged and can_merge:
            xG = self.get_merged_state(self)
        else:
            xG = self.get_continue_straight(self)
        K = self.nlmpc(x0, xG) # Controller intitialized and synthesized
        K.synthesize_mpc_controller()

        return control_input
    # Gets called from NLMPC, so the dynamics are re-worded
    # x: [x,y,v,theta]
    def dynamics_dubins(self, x, u):
        # if at max speed, cannot go faster
        x_next      = x[0] + self.dt * cos(x[3]) * x[2]
	    y_next      = x[1] + self.dt * sin(x[3]) * x[2]
	    v_next      = x[2] + self.dt * u[0]
	    theta_next  = x[3] + self.dt * u[1]

		state_next = [v_next, theta_next, x_next, y_next]

		return state_next

    def nlmpc(self, x0, xg):
        x0 = np.array([x0[2], x0[3], x[0], x[1]]) # np.arrays
        maxTime = 14 # Simulation time
        goal = np.array([x0[2], x0[3], x[0], x[1]]) # np.array's

        # Initialize mpc parameters
        N  = 20
        n = 4
        d = 2
        Q  = 1*np.eye(n)
        R  = 1*np.eye(d)
        Qf = 1000*np.eye(n)

    def synthesize_mpc_controller(self):
        # =================================================================
        # ======================== Subsection: Nonlinear MPC ==============
        # First solve the nonlinear optimal control problem as a Non-Linear Program (NLP)
        printLevel = 1
        xub = np.array([15, 15, 15, 15])
        uub = np.array([10, 0.5])
        nlp = NLP( N,  Q,  R,  Qf,  goal,  dt,  xub, self.uub, printLevel)
        ut  = nlp.solve(x0)

        #sys.reset_IC() # Reset initial conditions
        xPredNLP = []

        ut = []
        for t in range(0,maxTime): # Time loop
        	xt = [self.state[2], self.state[3], self.state[0], self.state[1]]
            self.xhist.append(xt)
        	ut = nlp.solve(xt)
        	xPredNLP.append(nlp.xPred)
            self.state = self.dynamics_dubins(xt, ut) # Updating car state

        x_cl_nlp = np.array(self.xhist)
        return xPredNLP

    def plot_control_input(self, xPredNLP):
        for timeToPlot in [0, 10]:
        	plt.figure()
        	plt.plot(xPredNLP[timeToPlot][:,0], xPredNLP[timeToPlot][:,1], '--.b', label="Predicted trajectory at time $t = $"+str(timeToPlot))
        	plt.plot(xPredNLP[timeToPlot][0,0], xPredNLP[timeToPlot][0,1], 'ok', label="$x_t$ at time $t = $"+str(timeToPlot))
        	plt.xlabel('$x$')
        	plt.ylabel('$y$')
        	plt.xlim(-1,12)
        	plt.ylim(-1,10)
        	plt.legend()

        plt.figure()
        for t in range(0, maxTime):
        	if t == 0:
        		plt.plot(xPredNLP[t][:,0], xPredNLP[t][:,1], '--.b', label='Predicted trajectory at time $t$')
        	else:
        		plt.plot(xPredNLP[t][:,0], xPredNLP[t][:,1], '--.b')
        plt.plot(x_cl_nlp[:,0], x_cl_nlp[:,1], '-*r', label="Closed-loop trajectory")
        plt.xlabel('$x$')
        plt.ylabel('$y$')
        plt.xlim(-1,12)
        plt.ylim(-1,10)
        plt.legend()
        plt.show()
