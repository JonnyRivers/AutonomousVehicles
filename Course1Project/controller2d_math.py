#!/usr/bin/env python3

"""
Math extensions for 2D Controller Class
"""

import numpy as np

def __heading_to_direction(heading):
    return np.sin(heading), np.cos(heading)

def normalize_heading(heading):
    if(heading > np.pi):
        heading = heading - (2 * np.pi)
    
    if(heading < -np.pi):
        heading = heading + (2 * np.pi)
    
    return heading

def direction_to_heading(x, y):
    heading = np.arctan2(-x, y) + (np.pi / 2)
    heading = normalize_heading(heading)
    
    return heading

def get_dp(a, b):
    a_x, a_y = __heading_to_direction(a)
    b_x, b_y = __heading_to_direction(b)

    dp = np.dot([a_x, a_y], [b_x, b_y])

    return dp