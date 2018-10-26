#!/usr/bin/env python

import numpy as np
import numpy.linalg as la
import rospy
from barc.msg import pos_info, ECU, prediction, SafeSetGlob
from trackInitialization import Map
import time


def main():
	rospy.init_node("something")
	node_name = rospy.get_name()
	this_agent = rospy.get_param(node_name + "/this_agent")
	other_agent = rospy.get_param(node_name + "/other_agent")

	track = Map()

	traj_predictor = Predictor(track)
	traj_collector = Trajectory(track)

	olSub = rospy.Subscriber(this_agent + "/OL_predictions", prediction,  traj_predictor.OLcallback)

	pub = rospy.Publisher(node_name + "/stuff", prediction, queue_size=1)
	# subscribe to stuff
	# publish inequality constraints
	startTime
	while(not rospy.is_shutdown()): 
		# do stuff: 
		pub.publish(traj_predictor.OL_predictions)
		# take collected tragectories from other car
		# take current state from other car
		# work out predicted trajectory

		# Get open loop predicted trajectory
		# formulate box constraints based on predicted trajectory 


class Predictor:
	def __init__(self, track):
		self.OL_predictions = prediction()
		self.track = track

	def OLcallback(self, predictions):
		self.OL_predictions = predictions

class Trajectory: 
	def __init__(self, track):
		self.track = track
		self.lap_num = 0
		self.half_flag = False
		self.current_position = pos_info()
		self.most_recent_trajectory = []

	def posCallback(pos_msg):
		self.current_position = pos_msg

	def updateTrajectories():
		if self.half_flag == True and self.current_position.s >= self.track.TrackLength:
			self.half_flag = False
			self.lap_num += 1
			# create new lap

		else: 
			# add position to trajectory
			if self.current_position.s >= self.track.TrackLength / 2: 
				self.half_flag = True
			
		

if __name__ == "__main__":

    try:    
        main()
        
    except rospy.ROSInterruptException:
        pass
