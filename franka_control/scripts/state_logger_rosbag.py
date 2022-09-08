#!/usr/bin/env python
'''
Publish any string (except "stop") to /filename to start recording.
Publish "stop" to /filename to stop recording.
'''
import rospy
import rosbag
from std_msgs.msg import String
from franka_msgs.msg import FrankaState

class FrankaListener:

  def __init__(self):
    rospy.init_node('franka_state_logger')
    self.recording_till_stop = False
    self.filename = ''
    self.f = None

  # Called each time there is a new message
  def stateCallback(self,data):
    if self.recording_till_stop:
      #for states to record, refer to https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html
      #cartesian velocity is written by us
      self.f.write('franka_state_controller', data)

  def filenameCallback(self,data):
    if data.data == 'stop' and self.recording_till_stop:
      self.recording_till_stop = False
      print("Stopping the recording")
      self.f.close()
      #STOP
    elif not self.recording_till_stop and data.data != 'stop':
      print("Starting to record, publish \"stop\" to stop recording")
      self.filename = data.data
      self.f = rosbag.Bag(self.filename+".bag", "w")
      self.recording_till_stop = True
      #START
    else:
      print("Bad command")

  # Setup the subscriber Node
  def listener(self):
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.stateCallback, queue_size=1000)
    rospy.Subscriber('filename', String, self.filenameCallback, queue_size=1000)
    rospy.spin()

if __name__ == '__main__':
  fr_listener = FrankaListener()
  fr_listener.listener()
