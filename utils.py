import numpy as np
import pdb

class system(object):
	"""docstring for system"""
	def __init__(self, x0, dt):
		self.x 	   = [x0]
		self.u 	   = []
		self.w 	   = []
		self.x0    = x0
		self.dt    = dt

	def applyInput(self, ut):
		self.u.append(ut)

		x = self.x[-1]
		x_next      = x[0] + self.dt * cos(x[3]) * x[2]
		y_next      = x[1] + self.dt * sin(x[3]) * x[2]
		v_next      = x[2] + self.dt * u[0]
		theta_next  = x[3] + self.dt * u[1]

		state_next = [x_next, y_next, v_next, theta_next]

		self.x.append(state_next)

	def reset_IC(self):
		self.x = [self.x0]
		self.u = []
		self.w = []
