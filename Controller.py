#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""
import numpy as np
import cutils

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
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

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

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
        x = self._current_x
        y = self._current_y
        yaw = self._current_yaw
        v = self._current_speed
        self.update_desired_speed()
        v_desired = self._desired_speed
        t = self._current_timestamp
        waypoints = self._waypoints
        throttle_output = 0
        steer_output = 0
        brake_output = 0

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
        self.vars.create_var('x_previous', 0.0)
        self.vars.create_var('y_previous', 0.0)
        self.vars.create_var('distance',0.0)
        self.vars.create_var('x_difference',0.0)
        self.vars.create_var('y_difference',0.0)
        self.vars.create_var('required_angle',0.0)
        self.vars.create_var('i',0)
        self.vars.create_var('j',0)
        self.vars.create_var('k',0)
        self.vars.create_var('fanglea',0.0)
        self.vars.create_var('fangleb',0.0)
        self.vars.create_var('fanglec',0.0)
        self.vars.create_var('fangled',0.0)
        self.vars.create_var('diff_1',0.0)
        self.vars.create_var('diff_2',0.0)
        self.vars.create_var('diff_3',0.0)
        self.vars.create_var('diff_4',0.0)
        self.vars.create_var('k_1',0)
        self.vars.create_var('k_2',0)
        self.vars.create_var('k_3',0)
        self.vars.create_var('k_4',0)
        self.vars.create_var('fangle',0.0)
        self.vars.create_var('diffangle',0.0)
        self.vars.create_var('prev_diffangle',0.0)
        self.vars.create_var('v_req_previous', 0.0)
        self.vars.create_var('steer_previous',0.0)
        self.vars.create_var('steer_put',0.0)
        self.vars.create_var('acceleration',0.0)
        self.vars.create_var('acceleration_previous',0.0)


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
            self.vars.x_difference = x-waypoints[self.vars.i][0]
            self.vars.y_difference = y-waypoints[self.vars.i][1]
            self.vars.distance = np.sqrt(np.square(self.vars.x_difference)+np.square(self.vars.y_difference))
            self.vars.acceleration = (np.sqrt(np.square(waypoints[self.vars.i+3][0]-waypoints[self.vars.i+2][0])+np.square(waypoints[self.vars.i+3][1]-waypoints[self.vars.i+2][1]))*(waypoints[self.vars.i+3][2]-waypoints[self.vars.i+3][2])/(waypoints[self.vars.i+3][2]+waypoints[self.vars.i+2][2]))
            #if (v-v_desired <= 0):
            throttle_output = np.minimum(np.maximum(((v_desired-v)*3.1 - (v-self.vars.v_previous)*6 + (v_desired - self.vars.v_req_previous)*12 + self.vars.distance*0.08 + 12000*self.vars.acceleration + 301000*(self.vars.acceleration-self.vars.acceleration_previous)),0),1)
            if (v-v_desired <= 0):
                if (v == 0):
                    #throttle_output = 0.5
                    #throttle_output = np.minimum(np.maximum((1+(v-v_desired)*0.0005 - (v-self.vars.v_previous)*5 + self.vars.distance*0.0001),0),1)
                    #throttle_output = np.minimum(np.maximum(((v_desired-v)*0.9 - (v-self.vars.v_previous)*5 + (v_desired - self.vars.v_req_previous)*6 + self.vars.distance*0.2),0),1)
                #else:
                    throttle_output = 1
            else:
                #throttle_output = np.minimum(np.maximum(((v_desired-v)*0.9 - (v-self.vars.v_previous)*5 + (v_desired - self.vars.v_req_previous)*6 + self.vars.distance*0.1),0),1)
                brake_output = np.minimum(np.maximum((-0.1*((v_desired-v)*0.9 - (v-self.vars.v_previous)*5 + (v_desired - self.vars.v_req_previous)*6 + self.vars.distance*0.1)),0),1)




            ######################################################
            ######################################################



            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            
            
            if (y-self.vars.y_previous > 0):
                self.vars.fangle = np.arctan((x-self.vars.x_previous)/(y-self.vars.y_previous))
            elif (y-self.vars.y_previous == 0):
                if (x > self.vars.x_previous):
                    self.vars.fangle = np.pi/2
                elif (x < self.vars.x_previous):
                    self.vars.fangle = -np.pi/2
                else:
                    self.vars.fangle = 0
            else:
                self.vars.fangle = np.arctan((x-self.vars.x_previous)/(y-self.vars.y_previous)) + np.pi



            if (waypoints[self.vars.i+2][1]-y > 0):
                self.vars.fanglea = np.arctan((waypoints[self.vars.i+2][0]-x)/(waypoints[self.vars.i+2][1]-y))
            elif (waypoints[self.vars.i+2][1]-y == 0):
                if (waypoints[self.vars.i+2][0]-x > 0):
                    self.vars.fanglea = np.pi/2
                elif (waypoints[self.vars.i+2][0]-x < 0):
                    self.vars.fanglea = -np.pi/2
                else:
                    self.vars.fanglea = 0
            else:
                self.vars.fanglea = np.arctan((waypoints[self.vars.i+2][0]-x)/(waypoints[self.vars.i+2][1]-y)) + np.pi



            if (waypoints[self.vars.i+4][1]-y > 0):
                self.vars.fangleb = np.arctan((waypoints[self.vars.i+4][0]-x)/(waypoints[self.vars.i+4][1]-y))
            elif (waypoints[self.vars.i+4][1]-y == 0):
                if (waypoints[self.vars.i+4][0]-x > 0):
                    self.vars.fangleb = np.pi/2
                elif (waypoints[self.vars.i+4][0]-x < 0):
                    self.vars.fangleb = -np.pi/2
                else:
                    self.vars.fangleb = 0
            else:
                self.vars.fangleb = np.arctan((waypoints[self.vars.i+4][0]-x)/(waypoints[self.vars.i+4][1]-y)) + np.pi



            if (waypoints[self.vars.i+5][1]-waypoints[self.vars.i+4][1] > 0):
                self.vars.fanglec = np.arctan((waypoints[self.vars.i+5][0]-waypoints[self.vars.i+4][0])/(waypoints[self.vars.i+5][1]-waypoints[self.vars.i+4][1]))
            elif (waypoints[self.vars.i+5][1]-waypoints[self.vars.i+4][1] == 0):
                if (waypoints[self.vars.i+5][0]-waypoints[self.vars.i+4][0] > 0):
                    self.vars.fanglec = np.pi/2
                elif (waypoints[self.vars.i+5][0]-waypoints[self.vars.i+4][0] < 0):
                    self.vars.fanglec = -np.pi/2
                else:
                    self.vars.fanglec = 0
            else:
                self.vars.fanglec = np.arctan((waypoints[self.vars.i+5][0]-waypoints[self.vars.i+4][0])/(waypoints[self.vars.i+5][1]-waypoints[self.vars.i+4][1])) + np.pi



            if (waypoints[self.vars.i+1][1]-waypoints[self.vars.i][1] > 0):
                self.vars.fangled = np.arctan((waypoints[self.vars.i+1][0]-waypoints[self.vars.i][0])/(waypoints[self.vars.i+1][1]-waypoints[self.vars.i][1]))
            elif (waypoints[self.vars.i+1][1]-waypoints[self.vars.i][1] == 0):
                if (waypoints[self.vars.i+1][0]-waypoints[self.vars.i][0] > 0):
                    self.vars.fangled = np.pi/2
                elif (waypoints[self.vars.i+1][0]-waypoints[self.vars.i][0] < 0):
                    self.vars.fangled = -np.pi/2
                else:
                    self.vars.fangled = 0
            else:
                self.vars.fangled = np.arctan((waypoints[self.vars.i+1][0]-waypoints[self.vars.i][0])/(waypoints[self.vars.i+1][1]-waypoints[self.vars.i][1])) + np.pi

            self.vars.k_1 = 0.0022
            self.vars.k_2 = 0.038
            self.vars.k_3 = 0.24
            self.vars.k_4 = 0.57


            self.vars.diff_1 = self.vars.fangle-self.vars.fanglea
            self.vars.diff_2 = self.vars.fangle-self.vars.fangleb
            self.vars.diff_3 = self.vars.fangle-self.vars.fanglec
            self.vars.diff_4 = self.vars.fangle-self.vars.fangled

            if (self.vars.fangle-self.vars.fanglea > np.pi):
                self.vars.diff_1 = -(2*np.pi - self.vars.fangle + self.vars.fanglea)
            elif (self.vars.fangle-self.vars.fanglea < -np.pi):
                self.vars.diff_1 = self.vars.fangle - self.vars.fanglea + 2*np.pi

            if (self.vars.fangle-self.vars.fangleb > np.pi):
                self.vars.diff_2 = -(2*np.pi - self.vars.fangle + self.vars.fangleb)
            elif (self.vars.fangle - self.vars.fangleb < -np.pi):
                self.vars.diff_2 = self.vars.fangle - self.vars.fangleb + 2*np.pi

            if (self.vars.fangle-self.vars.fanglec > np.pi):
                self.vars.diff_3 = -(2*np.pi - self.vars.fangle + self.vars.fanglec)
            elif (self.vars.fangle-self.vars.fanglec < -np.pi):
                self.vars.diff_3 = self.vars.fangle - self.vars.fanglec + 2*np.pi

            if (self.vars.fangle -self.vars.fangled > np.pi):
                self.vars.diff_4 = -(2*np.pi - self.vars.fangle + self.vars.fangled)
            elif (self.vars.fangle-self.vars.fangled < -np.pi):
                self.vars.diff_4 = self.vars.fangle - self.vars.fangled + 2*np.pi

            self.vars.steer_put = (self.vars.k_1*(self.vars.diff_1) + self.vars.k_2*(self.vars.diff_2) + self.vars.k_3*(self.vars.diff_3) + self.vars.k_4*(self.vars.diff_4))
            self.vars.diffangle = self.vars.fangle-self.vars.steer_previous
            steer_output = self.vars.steer_put + (self.vars.prev_diffangle - self.vars.diffangle)*3
            if (steer_output >= 1.22):
                steer_output = 1.22
            elif (steer_output <= -1.22):
                steer_output = -1.22

            ######################################################
            ######################################################


            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE 

            self.vars.v_previous = v
            self.vars.y_previous = y
            self.vars.x_previous = x
            self.vars.v_req_previous = v_desired
            self.vars.k = self.vars.i
            self.vars.acceleration_previous = self.vars.acceleration
            for self.vars.j in range(10):
                if self.vars.distance >= np.sqrt(np.square(x-waypoints[self.vars.i+self.vars.j-5][0])+np.square(y-waypoints[self.vars.i+self.vars.j-5][1])):
                    self.vars.k = self.vars.i + self.vars.j-5
            self.vars.i = self.vars.k
            self.vars.steer_previous = self.vars.fangle
            self.vars.prev_diffangle = self.vars.diffangle            
        ######################################################
        ######################################################

