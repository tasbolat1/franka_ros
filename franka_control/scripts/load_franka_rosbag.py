#!/usr/bin/env python

'''
This script plots state of first joint across time.
argv[1] takes in location of .bag file.
'''

import numpy as np
import matplotlib.pyplot as plt
import sys
import rosbag

bag = rosbag.Bag(sys.argv[1])

data_time = np.array([msg.header.stamp.secs + msg.header.stamp.nsecs * 10**-9 for topic, msg, t in bag.read_messages()])
data_q = np.array([msg.q for topic, msg, t in bag.read_messages()])
plt.plot(data_time,data_q[:,0])
plt.show()


