#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_x          = 0
        self._desired_y          = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def get_closest_waypoint_index(self, x, y):
        min_idx       = 0
        min_dist      = float("inf")
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - x,
                    self._waypoints[i][1] - y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        if min_idx < len(self._waypoints)-1:
            return min_idx
        
        return -1

    def get_heading_waypoint_index(self, closest_waypoint_index, v):
        min_heading_waypoint_distance = max(1, v) # distance traveled in one second
        closest_waypoint = self._waypoints[closest_waypoint_index]
        closest_waypoint_x = closest_waypoint[0]
        closest_waypoint_y = closest_waypoint[1]
        last_waypoint_to_check_index = len(self._waypoints)-1
        for i in range(closest_waypoint_index+1, last_waypoint_to_check_index-1):
            forward_waypoint = self._waypoints[i]
            forward_waypoint_x = forward_waypoint[0]
            forward_waypoint_y = forward_waypoint[1]
            distance_x = forward_waypoint_x - closest_waypoint_x
            distance_y = forward_waypoint_y - closest_waypoint_y
            distance_x_squared = distance_x * distance_x
            distance_y_squared = distance_y * distance_y
            distance_to_forward_waypoint = np.sqrt(distance_x_squared + distance_y_squared)
            if(distance_to_forward_waypoint > min_heading_waypoint_distance):
                return i
        
        return last_waypoint_to_check_index
    
    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints

        closest_waypoint_index = self.get_closest_waypoint_index(x, y)
        closest_waypoint = waypoints[closest_waypoint_index]
        v_desired = closest_waypoint[2]

        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('v_error_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            k_p = 2.5
            k_i = 1
            k_d = 2

            v_error = v_desired - v
            throttle_p = v_error * k_p
            #print(f"v_error = {v_error} ({v_desired} - {v})")
            
            v_error_integral = 0
            throttle_i = v_error_integral * k_i

            v_error_derivative = self.vars.v_error_previous - v_error
            throttle_d = v_error_derivative * k_d
            
            throttle_output = throttle_p + throttle_i + throttle_d
            throttle_output = min(throttle_output, 1)
            throttle_output = max(throttle_output, 0)
            #print(f"throttle: {throttle_output}; p: {throttle_p}; i: {throttle_i}; d: {throttle_d}")

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            #throttle_output = 0
            brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller. 
            steer_output = 0

            heading_waypoint_index = self.get_heading_waypoint_index(closest_waypoint_index, v)
            heading_waypoint = waypoints[heading_waypoint_index]

            # 1) Steer to align with desired heading
            closest_x = closest_waypoint[0]
            closest_y = closest_waypoint[1]
            heading_x = heading_waypoint[0]
            heading_y = heading_waypoint[1]
            x_e = heading_x - closest_x
            y_e = heading_y - closest_y

            # TODO fix the goofiness here
            yaw_desired = np.arctan2(-x_e, y_e) + (np.pi / 2)
            if(yaw_desired > np.pi):
                yaw_desired = yaw_desired - (2 * np.pi)
            yaw_e = yaw_desired - yaw
            if(yaw_e > np.pi):
                yaw_e = yaw_e - (2 * np.pi)
            if(yaw_e < -np.pi):
                yaw_e = yaw_e + (2 * np.pi)
            
            #print(f"yaw: {yaw}; yaw_desired: {yaw_desired}; closest({closest_x},{closest_y}); heading({heading_x},{heading_y}); pos_e({x_e}, {y_e})")

            steer_output = yaw_e

            # 2) Steer to eliminate crosstrack error
            # TODO

            crosstrack_e = 0
            distance_x = closest_waypoint[0] - x
            distance_y = closest_waypoint[1] - y
            distance_x_squared = distance_x * distance_x
            distance_y_squared = distance_y * distance_y
            distance_e = np.sqrt(distance_x_squared + distance_y_squared)

            print(f"distance_e: {distance_e}; v_e: {v_error}; yaw_e: {yaw_e}; crosstrack_e: {crosstrack_e}")

            # 3) Clamp
            steer_output = min(steer_output, 1.22)
            steer_output = max(steer_output, -1.22)

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.v_error_previous = v_error
