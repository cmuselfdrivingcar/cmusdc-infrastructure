#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

observation_length_4 = 12
observation_length_2 = 6
prediction_length = 20

frame = np.linspace(1, observation_length_4 + prediction_length, num = observation_length_4 + prediction_length)

x_true = np.zeros(observation_length_4 + prediction_length)
y_true = np.zeros(observation_length_4 + prediction_length)
x_pred_1 = np.zeros(observation_length_4 + prediction_length)
y_pred_1 = np.zeros(observation_length_4 + prediction_length)
x_pred_2 = np.zeros(observation_length_4 + prediction_length)
y_pred_2 = np.zeros(observation_length_4 + prediction_length)

x_pred = np.zeros(prediction_length)
y_pred = np.zeros(prediction_length)

shape_error = (prediction_length, 2)
error_4 = np.zeros(shape_error)
error_2 = np.zeros(shape_error)
error = np.zeros(shape_error)

polyfit_degree_1 = 4
polyfit_degree_2 = 2

x_true = np.cos(frame * np.pi / (observation_length_4 + prediction_length))
y_true = np.sin(frame * np.pi / (observation_length_4 + prediction_length))

p_x_1 = np.polyfit(frame[0:observation_length_4], x_true[0:observation_length_4], polyfit_degree_1)
p_y_1 = np.polyfit(frame[0:observation_length_4], y_true[0:observation_length_4], polyfit_degree_1)
x_pred_1[0:observation_length_4] = x_true[0:observation_length_4]
y_pred_1[0:observation_length_4] = y_true[0:observation_length_4]

p_x_2 = np.polyfit(frame[observation_length_4 - observation_length_2:observation_length_4], x_true[observation_length_4 - observation_length_2:observation_length_4], polyfit_degree_2)
p_y_2 = np.polyfit(frame[observation_length_4 - observation_length_2:observation_length_4], y_true[observation_length_4 - observation_length_2:observation_length_4], polyfit_degree_2)
x_pred_2[0:observation_length_4] = x_true[0:observation_length_4]
y_pred_2[0:observation_length_4] = y_true[0:observation_length_4]

p_x_1 = np.poly1d(p_x_1)
p_y_1 = np.poly1d(p_y_1)

p_x_2 = np.poly1d(p_x_2)
p_y_2 = np.poly1d(p_y_2)

for i in range(observation_length_4, observation_length_4 + prediction_length):
	x_pred_1[i] = p_x_1(frame[i])
	y_pred_1[i] = p_y_1(frame[i])
	error_4[i - observation_length_4][0] = x_pred_1[i] - x_true[i]
	error_4[i - observation_length_4][1] = y_pred_1[i] - y_true[i]
	
for i in range(observation_length_4, observation_length_4 + prediction_length):
	x_pred_2[i] = p_x_2(frame[i])
	y_pred_2[i] = p_y_2(frame[i])
	error_2[i - observation_length_4][0] = x_pred_2[i] - x_true[i]
	error_2[i - observation_length_4][1] = y_pred_2[i] - y_true[i]

x_pred = 0.5 * x_pred_1[observation_length_4: observation_length_4 + prediction_length] + 0.5 * x_pred_2[observation_length_4: observation_length_4 + prediction_length];
y_pred = 0.5 * y_pred_1[observation_length_4: observation_length_4 + prediction_length] + 0.5 * y_pred_2[observation_length_4: observation_length_4 + prediction_length];
error_x = x_pred - x_true[observation_length_4: observation_length_4 + prediction_length]
error_y = y_pred - y_true[observation_length_4: observation_length_4 + prediction_length]


print(error_4)
print "---------"
print(error_2)
print "---------"
print(error_x)
print(error_y)


_ = plt.plot(x_true, y_true, '--', x_pred, y_pred, '-', x_pred_1, y_pred_1, 'x', x_pred_2, y_pred_2, '.')
plt.show()


