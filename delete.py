import matplotlib.pyplot as plt
import numpy as np
t_ = 6000

# circle
x_  = []
y_  = []
alt = 0.5
x = np.linspace(0, 6, t_)
y = np.linspace(0, 6, t_)
time = np.linspace(0, 6, t_)

for i in xrange(0,t_):
	x_.append(0 + np.sin(time[i]))
	y_.append(0 + np.sin(time[i]))

plt.plot(x_, y_)
plt.show()