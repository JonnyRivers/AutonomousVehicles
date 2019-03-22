import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Vehicle():
    def __init__(self):
 
        # ==================================
        #  Parameters
        # ==================================
    
        #Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002
        
        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81
        
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01
        
        # Tire force 
        self.c = 10000
        self.F_max = 10000
        
        # State variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
        
        self.sample_time = 0.01
        
    def reset(self):
        # reset state variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
    
    def step(self, throttle, alpha):
        # integrate from previous frame
        self.x += (self.v * self.sample_time)
        self.v += (self.a * self.sample_time)
        self.w_e += (self.w_e_dot * self.sample_time)

        ##########################
        # calculate self.w_e_dot #
        ##########################

        # T_e (engine torque)
        T_e = throttle * (self.a_0 + self.a_1 * self.w_e + self.a_2 * self.w_e * self.w_e)

        # F_load (total load force)
        F_aero = self.c_a * self.v * self.v
        R_x = self.c_r1 * self.v
        F_g = self.m * self.g * np.sin(alpha)
        F_load = F_aero + R_x + F_g

        self.w_e_dot = (T_e - (self.GR * self.r_e * F_load)) / self.J_e

        # F_x (tyre force)
        w_w = self.GR * self.w_e
        s = ( (w_w * self.r_e - self.v) / self.v)
        F_x = self.F_max
        if(np.abs(s) < 1):
            F_x = self.c * s

        ####################
        # calculate self.a #
        ####################
        self.a = (F_x - F_load) / self.m

sample_time = 0.01
time_end = 20
model = Vehicle()
t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)

# reset the states
model.reset()

# ==================================
#  Learner solution begins here
# ==================================

throttle_data = np.zeros_like(t_data)
alpha_data = np.zeros_like(t_data)

def get_alpha(x):
    alpha = 0
    if(x < 60):
        alpha = np.tan(3/60)
    elif(x < 150):
        alpha = np.tan(9/90)
    
    return alpha

def get_throttle(t):
    if(t < 5):
        return 0.5 - (((5 - t) / 5) * 0.3)
    elif(t >=5 and t < 15):
        return 0.5
    elif(t >= 15 and t < 20):
        return (((20 - t) / 5) * 0.5)

for i in range(t_data.shape[0]):
    throttle = get_throttle(i * sample_time)
    alpha = get_alpha(model.x)

    x_data[i] =  model.x
    throttle_data[i] = throttle
    alpha_data[i] = alpha

    model.step(throttle, alpha)

# ==================================
#  Learner solution ends here
# ==================================
    
# Plot x vs t for visualization
plt.plot(t_data, x_data)
plt.show()