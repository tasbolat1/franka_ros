#!/usr/bin/env python

import roslib
import rospy
import os, sys
from types import *
import time
import numpy as np

from std_msgs.msg import String
from franka_msgs.msg import FrankaState


class FrankaListener:

  def __init__(self):
    rospy.init_node('franka_state_logger')
    self.recording_till_stop = False
    self.filename = ''
    self.frdata_matrix_singleframe = np.zeros((31)) #flattened array so we can save the array in a single line
    self.f = None

  # Called each time there is a new message
  def stateCallback(self,data):
    if self.recording_till_stop:
      #for states to record, refer to https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html
      #cartesian velocity is written by us
      print("Recorded Frame!")
      #states I am recording
      self.frdata_matrix_singleframe[0] = data.header.stamp.secs #time
      self.frdata_matrix_singleframe[1] = data.header.stamp.nsecs
      self.frdata_matrix_singleframe[2] = data.header.stamp.secs + data.header.stamp.nsecs * 10**-9
      self.frdata_matrix_singleframe[3:9] = data.cartesian_velocity[0:6] #cartesian velocity
      self.frdata_matrix_singleframe[9:15] = data.K_F_ext_hat_K[0:6] #force-torque relative to K frame
      self.frdata_matrix_singleframe[15:31] = data.O_T_EE[0:16] #cartesian pose
      #f.write(np.array2string(self.frdata_matrix_singleframe, max_line_width = 9999, precision = 15)+'\n')
      self.f.write(np.array2string(self.frdata_matrix_singleframe, max_line_width = 9999, precision = 15)[2:-1]+'\n')
      #f.write(str(np.around(self.frdata_matrix_singleframe,decimals=15)[1:-1])+'\n')


  def filenameCallback(self,data):
    if data.data == 'stop' and self.recording_till_stop:
      print("Stopping the recording")
      self.f.close()
      self.recording_till_stop = False
      #STOP
    elif not self.recording_till_stop and data.data != 'stop':
      print("Starting to record, publish \"stop\" to stop recording")
      self.recording_till_stop = True
      self.filename = data.data
      self.f = open(self.filename+".txt", "a")
      #START
    else:
      print("Bad command")

  # Setup the subscriber Node
  def listener(self):
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.stateCallback, queue_size=1000)
    rospy.Subscriber('filename', String, self.filenameCallback, queue_size=1000)
    rospy.spin()

  #def __del__(self):
    #self.fout.write("]")
    #self.fout.close()

if __name__ == '__main__':
  fr_listener = FrankaListener()
  fr_listener.listener()
