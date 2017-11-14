#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

frame = np.linspace(1, 20, num = 20)

x_true =  np.zeros(20)
y_true = np.zeros(20)
x_pred = np.zeros(20)
y_pred = np.zeros(20)

shape_error = (20, 2)
error = np.zeros(shape_error)

x_true = np.cos(frame * 9 * np.pi / 180)
y_true = np.sin(frame * 9 * np.pi / 180)

p_x = np.polyfit(frame[1:12], x_true[1:12], 4)
p_y = np.polyfit(frame[1:12], y_true[1:12], 4)
x_pred[0:12] = x_true[0:12]
y_pred[0:12] = y_true[0:12]

p_x = np.poly1d(p_x)
p_y = np.poly1d(p_y)

for i in range(12,20):
	x_pred[i] = p_x(frame[i])
	y_pred[i] = p_y(frame[i])
	error[i][0] = x_pred[i] - x_true[i]
	error[i][1] = y_pred[i] - y_true[i]

print(error)

_ = plt.plot(x_true, y_true, '--', x_pred, y_pred, '-')
plt.show()


