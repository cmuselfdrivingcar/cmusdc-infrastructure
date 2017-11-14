#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

observation_length = 12
prediction_length = 8

frame = np.linspace(1, observation_length + prediction_length, num = observation_length + prediction_length)

x_true =  np.zeros(observation_length + prediction_length)
y_true = np.zeros(observation_length + prediction_length)
x_pred = np.zeros(observation_length + prediction_length)
y_pred = np.zeros(observation_length + prediction_length)

shape_error = (observation_length + prediction_length, 2)
error = np.zeros(shape_error)

polyfit_degree = 3

x_true = np.cos(frame * 9 * np.pi / 180)
y_true = np.sin(frame * 9 * np.pi / 180)

p_x = np.polyfit(frame[0:observation_length], x_true[0:observation_length], polyfit_degree)
p_y = np.polyfit(frame[0:observation_length], y_true[0:observation_length], polyfit_degree)
x_pred[0:observation_length] = x_true[0:observation_length]
y_pred[0:observation_length] = y_true[0:observation_length]

p_x = np.poly1d(p_x)
p_y = np.poly1d(p_y)

for i in range(observation_length, observation_length + prediction_length):
	x_pred[i] = p_x(frame[i])
	y_pred[i] = p_y(frame[i])
	error[i][0] = x_pred[i] - x_true[i]
	error[i][1] = y_pred[i] - y_true[i]

print(error)

_ = plt.plot(x_true, y_true, '--', x_pred, y_pred, '-')
plt.show()


