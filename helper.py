import numpy as np
from numpy import sin, cos, tan

def find_corner_coordinates(x_state_center_before, y_state_center_before, x_desired, y_desired, theta, square_fig): # from Intersection Crossing code (Tung Phan)
    """
    This function takes an image and an angle then computes
    the coordinates of the corner (observe that vertical axis here is flipped).
    If we'd like to put the point specfied by (x_state_center_before, y_state_center_before) at (x_desired, y_desired),
    this function returns the coordinates of the lower left corner of the new image
    """
    w, h = square_fig.size
    theta = -theta
    if abs(w - h) > 1:
        print('Warning: Figure has to be square! Otherwise, clipping or unexpected behavior may occur')

    R = np.array([[cos(theta), sin(theta)], [-sin(theta), cos(theta)]])
    x_corner_center_before, y_corner_center_before = -w/2., -h/2. # lower left corner before rotation
    x_corner_center_after, y_corner_center_after = -w/2., -h/2. # doesn't change since figure size remains unchanged

    x_state_center_after, y_state_center_after = R.dot(np.array([[x_state_center_before], [y_state_center_before]])) # relative coordinates after rotation by theta

    x_state_corner_after = x_state_center_after - x_corner_center_after
    y_state_corner_after = y_state_center_after - y_corner_center_after

    x_corner_unknown = int(x_desired - x_state_center_after + x_corner_center_after)
    y_corner_unknown = int(y_desired - y_state_center_after + y_corner_center_after)
    return x_corner_unknown, y_corner_unknown
