#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

observation_length_high = 12
observation_length_low = 6
prediction_length = 20
dt = 0.1 
Ko = 0.3

frame = np.linspace(1, observation_length_high + prediction_length, num = observation_length_high + prediction_length)

x_true = np.zeros(observation_length_high + prediction_length)
y_true = np.zeros(observation_length_high + prediction_length)
x_pred_high = np.zeros(observation_length_high + prediction_length)
y_pred_high = np.zeros(observation_length_high + prediction_length)
x_pred_low = np.zeros(observation_length_high + prediction_length)
y_pred_low = np.zeros(observation_length_high + prediction_length)

x_pred = np.zeros(prediction_length)
y_pred = np.zeros(prediction_length)

shape_error = (prediction_length, 2)
error_high = np.zeros(shape_error)
error_low = np.zeros(shape_error)
error = np.zeros(shape_error)

polyfit_degree_1 = 4
polyfit_degree_2 = 2

x_true[0:7] = 1
y_true[0:7] = 1

x_true[8:observation_length_high + prediction_length] = np.cos(frame[8:observation_length_high + prediction_length] * np.pi / (observation_length_high + prediction_length))
y_true[8:observation_length_high + prediction_length] = np.sin(frame[8:observation_length_high + prediction_length] * np.pi / (observation_length_high + prediction_length))

###############################

#calculate x', x'', y', y''
vel_x = np.zeros(observation_length_high + prediction_length)
vel_y = np.zeros(observation_length_high + prediction_length)

acc_x = np.zeros(observation_length_high + prediction_length)
acc_y = np.zeros(observation_length_high + prediction_length)

vel_x[1:observation_length_high] = (x_true[1:observation_length_high] - x_true[0:observation_length_high - 1])/dt
vel_y[1:observation_length_high] = (y_true[1:observation_length_high] - y_true[0:observation_length_high - 1])/dt

acc_x[1:observation_length_high] = (vel_x[1:observation_length_high] - vel_x[0:(observation_length_high - 1)])/dt
acc_y[1:observation_length_high] = (vel_y[1:observation_length_high] - vel_y[0:observation_length_high - 1])/dt

#calculate K
prev_index = observation_length_high - observation_length_low - 1
curr_index = observation_length_high - 1
K_prev = (vel_x[prev_index] * acc_y[prev_index] - vel_y[prev_index] * acc_x[prev_index])/np.power((vel_x[prev_index] * vel_x[prev_index] + vel_y[prev_index] * vel_y[prev_index]), 1.5)
K_current = (vel_x[curr_index] * acc_y[curr_index] - vel_y[curr_index] * acc_x[curr_index])/np.power((vel_x[curr_index] * vel_x[curr_index] + vel_y[curr_index] * vel_y[curr_index]), 1.5)

print(K_prev)
print(K_current)

#if statements
if(np.fabs(K_prev - K_current) > Ko):
	if(K_prev - K_current > 0):
		#decrease the weights
		weight_lp = 0.75
		weight_hp = 0.25
	else:
		weight_lp = 0.25
		weight_hp = 0.75
else:
	weight_lp = 0.5
	weight_hp = 0.5

################################

p_x_1 = np.polyfit(frame[0:observation_length_high], x_true[0:observation_length_high], polyfit_degree_1)
p_y_1 = np.polyfit(frame[0:observation_length_high], y_true[0:observation_length_high], polyfit_degree_1)
x_pred_high[0:observation_length_high] = x_true[0:observation_length_high]
y_pred_high[0:observation_length_high] = y_true[0:observation_length_high]

p_x_2 = np.polyfit(frame[observation_length_high - observation_length_low:observation_length_high], x_true[observation_length_high - observation_length_low:observation_length_high], polyfit_degree_2)
p_y_2 = np.polyfit(frame[observation_length_high - observation_length_low:observation_length_high], y_true[observation_length_high - observation_length_low:observation_length_high], polyfit_degree_2)
x_pred_low[0:observation_length_high] = x_true[0:observation_length_high]
y_pred_low[0:observation_length_high] = y_true[0:observation_length_high]

p_x_1 = np.poly1d(p_x_1)
p_y_1 = np.poly1d(p_y_1)

p_x_2 = np.poly1d(p_x_2)
p_y_2 = np.poly1d(p_y_2)

for i in range(observation_length_high, observation_length_high + prediction_length):
	x_pred_high[i] = p_x_1(frame[i])
	y_pred_high[i] = p_y_1(frame[i])
	error_high[i - observation_length_high][0] = x_pred_high[i] - x_true[i]
	error_high[i - observation_length_high][1] = y_pred_high[i] - y_true[i]
	
for i in range(observation_length_high, observation_length_high + prediction_length):
	x_pred_low[i] = p_x_2(frame[i])
	y_pred_low[i] = p_y_2(frame[i])
	error_low[i - observation_length_high][0] = x_pred_low[i] - x_true[i]
	error_low[i - observation_length_high][1] = y_pred_low[i] - y_true[i]

x_pred = weight_hp * x_pred_high[observation_length_high: observation_length_high + prediction_length] + weight_lp * x_pred_low[observation_length_high: observation_length_high + prediction_length];
y_pred = weight_hp * y_pred_high[observation_length_high: observation_length_high + prediction_length] + weight_lp * y_pred_low[observation_length_high: observation_length_high + prediction_length];
error_x = x_pred - x_true[observation_length_high: observation_length_high + prediction_length]
error_y = y_pred - y_true[observation_length_high: observation_length_high + prediction_length]



print(error_high)
print "---------"
print(error_low)
print "---------"
print(error_x)
print(error_y)


_ = plt.plot(x_true, y_true, '--', x_pred, y_pred, '-', x_pred_high, y_pred_high, 'x', x_pred_low, y_pred_low, '.')
plt.show()


