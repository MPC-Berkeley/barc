#!/usr/bin/env python

import os
import sys
import copy
sys.path.append(sys.path[0]+'/ControllersObject')
sys.path.append(sys.path[0]+'/Utilities')
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
	frequency = rospy.get_param(node_name + "/frequency")
	dt = 1/frequency

	track = Map()

	traj_predictor = Predictor(track)
	traj_collector = Trajectory(track)

	olSub = rospy.Subscriber(this_agent + "/OL_predictions", prediction,  traj_predictor.OLcallback)
	trajSub = rospy.Subscriber(other_agent + "/pos_info", pos_info, traj_collector.posCallback)

	pub = rospy.Publisher(this_agent + "/other_agent_predictions", prediction, queue_size=1)
	# subscribe to stuff
	# publish inequality constraints
	startTime = time.time()
	other_pred = prediction()

	while(not rospy.is_shutdown()):

		if time.time() - startTime >= dt: 
			# @ 10 HERTZ:
			startTime = time.time()
			# Collect trajectories from the other car
			traj_collector.updateTrajectories()
			if traj_collector.lap_num >= 2: 
				pred = traj_collector.getPrediction(traj_predictor.horizon)
				other_pred.s = pred[0,:]
				other_pred.ey = pred[1,:]
				other_pred.epsi = pred[2,:]
				pub.publish(other_pred)
			

			# take collected tragectories from other car
			# take current state from other car
			# work out predicted trajectory

			# Get open loop predicted trajectory
			# formulate box constraints based on predicted trajectory 


class Predictor:
	def __init__(self, track):
		self.OL_predictions = prediction()
		self.track = track
		self.horizon = 0

	def OLcallback(self, predictions):
		self.OL_predictions = predictions
		self.horizon = len(predictions.s)

class Trajectory: 
	def __init__(self, track):
		self.track = track
		self.lap_num = 0
		self.half_flag = False
		self.first_quad_flag = False
		self.current_position = pos_info()
		self.most_recent_trajectory = []
		self.first_quad_traj = []
		self.current_trajectory = []

	def posCallback(self, pos_msg):
		self.current_position = pos_msg

	def getCurrentPosition(self):
		state = np.array(self.track.getLocalPosition(
			self.current_position.x,
			self.current_position.y, 
			self.current_position.psi))
		return state[0:-1]

	def updateTrajectories(self):
		state = self.getCurrentPosition()

		if self.half_flag == True and state[0] < self.track.TrackLength / 4:
			self.half_flag = False
			self.first_quad_flag = False
			self.lap_num += 1
			# create new lap
			self.most_recent_trajectory = copy.copy(self.current_trajectory)
			self.current_trajectory = []
			self.current_trajectory.append(state)			

		else: 
			# add position to trajectory
			self.current_trajectory.append(state)

			if state[0] >= self.track.TrackLength / 4 and self.first_quad_flag == False: 
				self.first_quad_flag = True
				self.first_quad_traj = copy.copy(self.current_trajectory)

			if state[0] >= self.track.TrackLength / 2 and self.half_flag == False: 
				self.half_flag = True

	def getTrajectory(self):
		s = self.getCurrentPosition()[0] 
		if s >= self.track.TrackLength * (0.75): 
			traj = self.most_recent_trajectory + self.first_quad_traj
			return traj
		else:
			return self.most_recent_trajectory

	def getPrediction(self, horizon): 
		traj = self.getTrajectory()
		traj = np.array(traj)
		state = np.transpose(self.getCurrentPosition())
		norms = np.linalg.norm(traj - state, axis = 1)
		start_index = np.argmin(norms)
		traj = np.transpose(traj)
		pred = traj[:, start_index + 1 :start_index + horizon +1]
		# print("number of states in traj = " + str(traj.shape))
		# print("index bounds = " + str(start_index) + ", " + str(start_index + horizon))
		# print("shape of prediction array" + str(pred.shape))
		# print("horizon length in pred = " + str(horizon))
		# if pred.shape[1] == 0:
		# 	print("BOOOOO!!!!")
		return pred

if __name__ == "__main__":

    try:    
        main()
        
    except rospy.ROSInterruptException:
        pass
