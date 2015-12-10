#!/usr/bin/python
from math import pi,sin
import serial

# communication scheme
# ODROID -> Arduino -> actuators (servo, motor)
class Controller:
	def __init__(self, arduino_port = "/dev/ttyUSB0", baudRate = 115200, sampleRate = 10, radius = 1):
		# set-up communication with arduino
		self.port 			= serial.Serial(port = arduino_port, baudrate = baudRate)
		if self.port.isOpen():
			self.port.close()
		self.port.open()
			
		# Electronic Speed Control (ESC) variables 
		self.target_FxR_PWM = 99
		self.brake 			= 50
		self.neutral 		= 90
		self.stopped 		= False

		# Servo input variables
		self.df_PWM 		= 0
		self.L_turn 		= self.angle_2_servo(20) 	# left turn
		self.R_turn 		= self.angle_2_servo(-20) 	# right turn
		self.Z_turn 		= self.angle_2_servo(0)	# zero turn
			
		# Encoder variables
		self.enc_count_k 	= 0 		# encoder count
		self.enc_count_km1 	= 0			# encoder count at next previous time step (k minus one) 
		self.vx_enc 		= 0
		self.sampleRate 	= sampleRate
		self.radius 		= radius

		# initialize the run
		self.send_cmd(self.neutral, self.Z_turn)

	################################################################
	# Steering Wheel and Motor Functions
	################################################################
	# [deg] -> [PWM]
	def angle_2_servo(self, x):
		u   = 92.0558 + 1.8194*x  - 0.0104*x**2
		return u

	# [PWM] -> [deg]
	def servo_2_angle(self, x):
		df   = 39.2945 + 0.3018*x  - 0.0014*x**2
		return df
	
	# longitudinal force / steering wheel command
	def send_cmd(self, FxR_PWM = None, df_PWM = None):
		# go straight and the target velocity by default
		if FxR_PWM == None:
			FxR_PWM = self.target_FxR_PWM
		if df_PWM == None:
			df_PWM = self.Z_turn

		# send command to arduino
		cmd = str(int(FxR_PWM)) +','+ str(int(df_PWM)) + 'f' 
		print cmd
		self.port.write( cmd )
	
	# turn left
	def turnLeft(self):
		self.df_PWM 		= self.L_turn
		self.send_cmd(self.target_FxR_PWM, self.df_PWM)

	# turn right
	def turnRight(self):
		self.df_PWM 	= self.R_turn
		self.send_cmd(self.target_FxR_PWM, self.df_PWM)
	
	# go straight
	def goStraight(self):
		self.df_PWM		= self.Z_turn
		self.send_cmd(self.target_FxR_PWM, self.df_PWM)

	# turn on brakes
	def setBrake(self):
		self.df_PWM		= self.Z_turn
		if not self.stopped:
			self.send_cmd(self.brake, self.df_PWM)
			self.stopped = True
		else:
			self.setNeutral()

	# go neutral
	def setNeutral(self):
		self.df_PWM		= self.Z_turn
		self.send_cmd(self.neutral, self.df_PWM)

	def hasStopped(self):
		return self.stopped

	################################################################
	# Encoder Functions
	################################################################
	def readEncoder(self):
		# read data
		raw_data  	= self.port.inWaiting()
		parse_data 	= self.port.read(raw_data).splitlines()

		# get new encoder count
		if(len(data) > 1):
			self.enc_count_k 	= int(parse_data[-2])
		else:
			self.enc_count_k 	= self.enc_count_km1

		# compute v_x
		self.vx_enc 	= (self.enc_count_k - self.enc_count_km1)*self.radius*self.sampleRate / 8


