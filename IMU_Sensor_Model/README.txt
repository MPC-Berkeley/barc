File:         IMU_sensor_model

This sensor model takes in acceleration and rotation data from the IMU sensor,
and filter the signal to produce estimates of longitudinal and lateral velocity.,
v_x and v_y respectively

The file conains one class, signal, and one function, estimate_position

Class: signal
Member functions
	Constructor: ( with following key word arguments) 
	Initializes object and sets appropriate filter
		[float, list] 	y0  			:=  initial signal (usually set to zero)
		[float, list] 	a   			:=  tuning parameter for the low pass filter,  (heavy filter)  0 <= a <= 1 (no filter)
		[int]    			n  			:=  length of memory block for moving average filter
		[string] 		method 	:= low pass filter (lpf), 
												moving average filter (mvg),
												weighted moving average filter (wmvg)
												No filter (None)
	
	update( y_new)
	Runs filter through filter (if selected), and saves result internally
	
	getSignal( option )
	Returns raw or filtered signal
	[string] option 				:= 'raw' or 'fflt'
	
function: estimate_position
	Input: current state estimate (x0), filtered signal data object (Y_out) ,magnitude of time step (dt)
	return: state estimate at next time step
	
function compute_yaw_rate
	Input: 		Yaw angle measurements from current and previous time step, and magnitude time step
	Output: 	Estimate of yaw rate