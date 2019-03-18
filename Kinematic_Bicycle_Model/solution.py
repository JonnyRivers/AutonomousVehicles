import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Bicycle():
    def __init__(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0

        self.L = 2
        self.lr = 1.2
        self.w_max = 1.22

        self.sample_time = 0.01

    def reset(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0

    def step(self, v, w):
        print("INPUTS: v=%f; w=%f" % (v, w))

        if(w > self.w_max):
            w = self.w_max
        
        if(w < -self.w_max):
            w = -self.w_max
        
        print("VALIDATED INPUTS: v=%f; w=%f" % (v, w))

        dot_delta = w
        self.delta = self.delta + dot_delta * self.sample_time

        self.beta = np.arctan((self.lr * np.tan(self.delta)) / self.L)

        dot_theta = ((v * np.cos(self.beta) * np.tan(self.delta)) / self.L)
        self.theta = self.theta + dot_theta * self.sample_time

        dot_xc = v * np.cos(self.theta + self.beta)
        self.xc = self.xc + dot_xc * self.sample_time

        dot_yc = v * np.sin(self.theta + self.beta)
        self.yc = self.yc + dot_yc * self.sample_time

        print("INTERMEDIATES: dot_delta=%f; dot_theta=%f; dot_xc=%f; dot_yc=%f" % (dot_delta, dot_theta, dot_xc, dot_yc))

        print("OUTPUTS: xc=%f; yc=%f; theta=%f; delta=%f; beta=%f" % (self.xc, self.yc, self.theta, self.delta, self.beta))


model = Bicycle()
sample_time = 0.01
time_end = 30

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
w_data = np.zeros_like(t_data)

length = 2 * 2 * np.pi * 8 * 0.9887
v = length / time_end
v_data[:] = v

target_delta = np.arctan(2/(8*0.9887))

for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc

    if(i > 353 and i <= 1831):
        if model.delta > -target_delta:
            w = min(-model.w_max, (target_delta + model.delta) / sample_time)
            w_data[i] = w
        else:
            w_data[i] = 0
    else:
        if model.delta < target_delta:
            w = min(model.w_max, (target_delta - model.delta) / sample_time)
            w_data[i] = w
        else:
            w_data[i] = 0
    model.step(v_data[i], w_data[i])

plt.axis('equal')
plt.plot(x_data, y_data)
plt.show()