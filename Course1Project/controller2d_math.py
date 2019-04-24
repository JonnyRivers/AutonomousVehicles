#!/usr/bin/env python3

"""
Math extensions for 2D Controller Class
"""

import numpy as np

def heading_to_direction(heading):
    heading = normalize_heading(heading - np.pi / 2)
    return [-np.sin(heading), np.cos(heading)]

def normalize_heading(heading):
    if(heading > np.pi):
        heading = heading - (2 * np.pi)
    
    if(heading < -np.pi):
        heading = heading + (2 * np.pi)
    
    return heading

def direction_to_heading(direction):
    heading = np.arctan2(-direction[0], direction[1]) + (np.pi / 2)
    heading = normalize_heading(heading)
    
    return heading

def get_dp(a_heading, b_heading):
    a_direction = heading_to_direction(a_heading)
    b_direction = heading_to_direction(b_heading)

    dp = np.dot(a_direction, b_direction)

    return dp

def vec2d_subtract(a, b):
    return [a[0] - b[0], a[1] - b[1]]

def vec2d_distance(a, b):
    return np.linalg.norm(np.array([
        b[0] - a[0],
        b[1] - a[1]]))