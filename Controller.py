            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            self.vars.x_difference = x-waypoints[self.vars.i][0]
            self.vars.y_difference = y-waypoints[self.vars.i][1]
            self.vars.distance = np.sqrt(np.square(self.vars.x_difference)+np.square(self.vars.y_difference))
            throttle_output = np.minimum(np.maximum(((v_desired-v)*1.1 - (v-self.vars.v_previous)*5 + (v_desired - self.vars.v_req_previous)*6 + self.vars.distance*0.2),0),1)
            if (v-v_desired <= 0):
                if (v == 0):
                    throttle_output = 1
            else:
                brake_output = np.minimum(np.maximum((-0.1*((v_desired-v)*0.9 - (v-self.vars.v_previous)*5 + (v_desired - self.vars.v_req_previous)*6 + self.vars.distance*0.1)),0),1)
            ######################################################

            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE


            #Initializing the required angles for steering control

            #Angles hold the value -pi/2 to 3pi/2
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

            #Setting the value of weight coefficients
            self.vars.k_1 = 0.006
            self.vars.k_2 = 0.018
            self.vars.k_3 = 0.08
            self.vars.k_4 = 0.04

            self.vars.diff_1 = self.vars.fangle-self.vars.fanglea
            self.vars.diff_2 = self.vars.fangle-self.vars.fangleb
            self.vars.diff_3 = self.vars.fangle-self.vars.fanglec
            self.vars.diff_4 = self.vars.fangle-self.vars.fangled


            #setting the values of required angles
            if (self.vars.fangle-self.vars.fanglea > np.pi):
                self.vars.diff_1 = -(2*np.pi - self.vars.fangle + self.vars.fanglea)
            elif (self.vars.fangle-self.vars.fanglea < -np.pi):
                self.vars.diff_1 = self.vars.fangle - self.vars.fanglea + 2*np.pi

            if (self.vars.fangle-self.vars.fangleb > np.pi):
                self.vars.diff_2 = -(2*np.pi - self.vars.fangle + self.vars.fangleb)
            elif (self.vars.fangle-self.vars.fangleb < -np.pi):
                self.vars.diff_2 = self.vars.fangle - self.vars.fangleb + 2*np.pi

            if (self.vars.fangle-self.vars.fanglec > np.pi):
                self.vars.diff_3 = -(2*np.pi - self.vars.fangle + self.vars.fanglec)
            elif (self.vars.fangle-self.vars.fanglec < -np.pi):
                self.vars.diff_3 = self.vars.fangle - self.vars.fanglec + 2*np.pi

            if (self.vars.fangle -self.vars.fangled > np.pi):
                self.vars.diff_4 = -(2*np.pi - self.vars.fangle + self.vars.fangled)
            elif (self.vars.fangle-self.vars.fangled < -np.pi):
                self.vars.diff_4 = self.vars.fangle - self.vars.fangled + 2*np.pi

            #setting the steer output
            self.vars.steer_put = (self.vars.k_1*(self.vars.diff_1) + self.vars.k_2*(self.vars.diff_2) + self.vars.k_3*(self.vars.diff_3) + self.vars.k_4*(self.vars.diff_4))

            steer_output = self.vars.steer_put - (self.vars.fangle-self.vars.steer_previous)*4
            if (steer_output >= 1.22):
                steer_output = 1.22
            elif (steer_output <= -1.22):
                steer_output = -1.22
            ######################################################

            #Refreshing the previous values of variables for the next loop
            self.vars.v_previous = v
            self.vars.y_previous = y
            self.vars.x_previous = x
            self.vars.v_req_previous = v_desired
            self.vars.k = self.vars.i
            for self.vars.j in range(10):
                if self.vars.distance >= np.sqrt(np.square(x-waypoints[self.vars.i+self.vars.j-5][0])+np.square(y-waypoints[self.vars.i+self.vars.j-5][1])):
                    self.vars.k = self.vars.i + self.vars.j-5
            self.vars.i = self.vars.k

            self.vars.steer_previous = self.vars.fangle

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)
