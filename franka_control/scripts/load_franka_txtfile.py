#!/usr/bin/env python

'''
This script plots force on end effector on x-axis of end effector.
'''

import numpy as np
import matplotlib.pyplot as plt
import sys

with open(sys.argv[1]) as f:
    content = f.readlines()
# you may also want to remove whitespace characters like `\n` at the end of each line
#content = [x.strip() for x in content]
franka_over_time = np.zeros((len(content),31))
for i in range(len(content)):
	x = content[i][:-1].split(' ')
	franka_over_time[i,:] = np.array([a for a in x if a != ''])

print("This script plots force on end effector on x-axis of end effector.")
print(franka_over_time.shape)
print(franka_over_time[0,2])
print(franka_over_time[1,2])


plt.plot(franka_over_time[:,2]-franka_over_time[0,2],franka_over_time[:,3], label = 'dx')
plt.plot(franka_over_time[:,2]-franka_over_time[0,2],franka_over_time[:,4], label = 'dy')
plt.plot(franka_over_time[:,2]-franka_over_time[0,2],franka_over_time[:,5], label = 'dz')
plt.plot(franka_over_time[:,2]-franka_over_time[0,2],franka_over_time[:,27], label = 'x')
plt.plot(franka_over_time[:,2]-franka_over_time[0,2],franka_over_time[:,28], label = 'y')
plt.plot(franka_over_time[:,2]-franka_over_time[0,2],franka_over_time[:,29], label = 'z')
plt.legend(loc='upper right')
plt.show()

