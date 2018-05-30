#!/usr/bin/env julia

#=
	File name: optimizer.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

using RobotOS
using JuMP
using Ipopt

include("agent.jl")
include("track.jl")


type Optimizer
	model::JuMP.Model

	init_state_s::Array{JuMP.NonlinearParameter, 1}
	current_input::Array{JuMP.NonlinearParameter, 1}

	states_s::Array{JuMP.Variable, 2}
	inputs::Array{JuMP.Variable, 2}
	alpha::Array{JuMP.Variable}

	weights_derivative_states::Array{Float64, 1}
	weights_derivative_inputs::Array{Float64, 1}
	weights_states::Array{Float64, 1}
	weights_inputs::Array{Float64, 1} 
	weights_lane::Float64
	weights_velocity::Float64
	weights_obstacles::Array{JuMP.NonlinearParameter, 1}
	weights_progress::Array{Float64, 1}
	weights_obstacle::JuMP.NonlinearParameter
	
	derivative_cost::JuMP.NonlinearExpression
	path_cost::JuMP.NonlinearExpression
	control_cost::JuMP.NonlinearExpression
	lane_cost::JuMP.NonlinearExpression
	velocity_cost::JuMP.NonlinearExpression
	terminal_cost::JuMP.NonlinearExpression
	obstacle_cost::JuMP.NonlinearExpression
	progress_cost::JuMP.NonlinearExpression

	curvature::Array{JuMP.NonlinearParameter, 1}
	reference_s::Array{JuMP.NonlinearParameter, 2}
	progress::Array{JuMP.NonlinearParameter, 1}

	theta_vx::Array{JuMP.NonlinearParameter, 1}
	theta_vy::Array{JuMP.NonlinearParameter, 1}
	theta_psi_dot::Array{JuMP.NonlinearParameter, 1}

	selected_states::Array{JuMP.NonlinearParameter, 2}
	states_cost::Array{JuMP.NonlinearParameter, 1}

	solution_inputs::Array{Float64}
	solution_states_s::Array{Float64}

	solver_status::Symbol

	agent::Agent
	adv_states_s::Array{JuMP.NonlinearParameter, 2}

	adv_predictions_s::Array{Float64}
	adv_current_lap::Int64
	predictions_s::prediction
	first_input::ECU
	adversarial::Bool
	prediction_sub::RobotOS.Subscriber{barc.msg.prediction}
	prediction_pub::RobotOS.Publisher{barc.msg.prediction}
	input_pub::RobotOS.Publisher{barc.msg.ECU}

	Optimizer() = new()
end

function init!(optimizer::Optimizer, agent::Agent, horizon::Int64)
	optimizer.agent = Agent()
	optimizer.agent = agent  # use the agent from the simulator
		
	optimizer.solution_inputs = zeros(horizon, 2)
	optimizer.solution_states_s = zeros(horizon + 1, 4)

	num_considered_states = size(optimizer.agent.selected_states_s)[1]

	optimizer.weights_derivative_inputs = 0.1 * [1.0, 10.0]
	optimizer.weights_inputs = 2.0 * [1.0, 1.0]
	optimizer.weights_lane = 10.0 * 1.0
	optimizer.weights_velocity = 1.0
	# optimizer.weights_obstacles = ones(num_considered_states)
	# optimizer.weights_progress = - 300.0 * ones(horizon + 1)
	optimizer.weights_progress = 0.5 * (- 1.0 * ones(horizon + 1))

	optimizer.predictions_s = prediction()
	optimizer.first_input = ECU()
	optimizer.adv_predictions_s = zeros(horizon + 1, 6)
	optimizer.adv_predictions_s[:, 1] = 0.1 * rand(horizon + 1) + 10 * agent.index
	optimizer.adv_current_lap = 0
	optimizer.adversarial = false
	dummy = 0
	optimizer.prediction_sub = Subscriber("adv_prediction", prediction, 
										  adv_prediction_callback, 
										  (optimizer, dummy), 
										  queue_size=1)::RobotOS.Subscriber{barc.msg.prediction}
	optimizer.prediction_pub = Publisher("prediction", prediction, 
										 queue_size=1)::RobotOS.Publisher{barc.msg.prediction}
	# optimizer.input_pub = Publisher("ecu", ECU, 
	#								queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
end

function adv_prediction_callback(msg::prediction, optimizer::Optimizer, dummy)
	optimizer.adv_predictions_s[:, 1] = msg.s
	optimizer.adv_predictions_s[:, 2] = msg.ey
	optimizer.adv_predictions_s[:, 3] = msg.epsi
	optimizer.adv_predictions_s[:, 4] = msg.psidot
	optimizer.adv_predictions_s[:, 5] = msg.vx
	optimizer.adv_predictions_s[:, 6] = msg.vy
	optimizer.adv_current_lap = msg.current_lap
	optimizer.adversarial = true
end

function publish_prediction(optimizer::Optimizer)
	optimizer.predictions_s.header.stamp = get_rostime()
	if size(optimizer.solution_states_s, 2) == 6
		optimizer.predictions_s.s = optimizer.solution_states_s[:, 1]
		optimizer.predictions_s.ey = optimizer.solution_states_s[:, 2]
		optimizer.predictions_s.epsi = optimizer.solution_states_s[:, 3]
		optimizer.predictions_s.psidot = optimizer.solution_states_s[:, 4]
		optimizer.predictions_s.vx = optimizer.solution_states_s[:, 5]
		optimizer.predictions_s.vy = optimizer.solution_states_s[:, 6]
	else
		optimizer.predictions_s.s = optimizer.solution_states_s[:, 1]
		optimizer.predictions_s.ey = optimizer.solution_states_s[:, 2]
		optimizer.predictions_s.epsi = optimizer.solution_states_s[:, 3]
		optimizer.predictions_s.psidot = zeros(optimizer.solution_states_s[:, 3])
		optimizer.predictions_s.vx = optimizer.solution_states_s[:, 4]
		optimizer.predictions_s.vy = zeros(optimizer.solution_states_s[:, 4])
	end
	optimizer.predictions_s.current_lap = optimizer.agent.current_lap

	# optimizer.first_input.motor = optimizer.solution_inputs[1, 1]
	# optimizer.first_input.servo = optimizer.solution_inputs[1, 2]

    publish(optimizer.prediction_pub, optimizer.predictions_s)
    # publish(optimizer.input_pub, optimizer.first_input)
end

function init_path_following!(optimizer::Optimizer, track::Track)
	# TODO: refactor init_path_following and init_learning_mpc

	optimizer.weights_derivative_states = 0.1 * [0.0, 0.0, 0.1, 0.1] 
	optimizer.weights_states = 1.0 * [0.0, 10.0, 0.1, 1.0]

	weights_derivative_states = optimizer.weights_derivative_states
	weights_derivative_inputs = optimizer.weights_derivative_inputs
	weights_states = optimizer.weights_states
	weights_inputs = optimizer.weights_inputs

	dt = optimizer.agent.dt
	# println("optimiziation dt: ", dt)
	horizon = size(optimizer.agent.optimal_inputs)[1]
	l_front = optimizer.agent.l_front
	l_rear = optimizer.agent.l_rear
	
	iteration = optimizer.agent.current_iteration
	initial_s = get_current_s(optimizer.agent)
	kappa = zeros(horizon)  # curvature
	
	optimizer.model = Model(solver=IpoptSolver(print_level=0, linear_solver="ma27",
											   max_cpu_time=0.09))

	@NLparameter(optimizer.model, reference_s[i = 1 : (horizon + 1), 
				 j = 1 : 4] == 0)
	reference_input = zeros(horizon, 2)
	
	@variable(optimizer.model, optimizer.agent.state_s_lower_bound[j] <= 
			  optimizer.states_s[i = 1 : (horizon + 1), j = 1 : 4] <=
  			  optimizer.agent.state_s_upper_bound[j])
	@variable(optimizer.model, optimizer.agent.input_lower_bound[j] <= 
			  optimizer.inputs[i = 1 : horizon, j = 1 : 2] <= 
			  optimizer.agent.input_upper_bound[j])

	states_s = optimizer.states_s
	inputs = optimizer.inputs
	# println(getvalue(states_s[1]))
	
	@NLparameter(optimizer.model, optimizer.current_input[i = 1 : 2] == 0)
	@NLparameter(optimizer.model, optimizer.init_state_s[i = 1 : 4] == 
				 initial_s[i])
	@NLconstraint(optimizer.model, [i = 1 : 4], 
				  states_s[1, i] == optimizer.init_state_s[i])

	# TODO: What's up with the track curvature in the optimization
	# get_curvature does not work in optimizaion. Using different
	# approach based on an estimate. Although exact curvature would be 
	# available
	# estimate curvature

	@NLparameter(optimizer.model, curvature[i = 1 : horizon] == kappa[i])

	@NLexpression(optimizer.model, beta[i = 1 : horizon], atan(l_rear / 
				  (l_front + l_rear) * tan(inputs[i, 2])))
	@NLexpression(optimizer.model, state_s_dot[i = 1 : horizon], 
				  states_s[i, 4] * cos(states_s[i, 3] + 
				  beta[i]) / (1 - states_s[i, 2] * curvature[i]))

	# Kinematic model
	#=
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 1] == states_s[i, 1] + dt * state_s_dot[i])
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 2] == states_s[i, 2] + 
				  dt * states_s[i, 4] * sin(states_s[i, 3] + beta[i]))
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 3] == states_s[i, 3] + 
				  dt * (states_s[i, 4] / l_rear * 
				  sin(beta[i]) - state_s_dot[i] * curvature[i]))
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 4] == states_s[i, 4] + dt * inputs[i, 1])
	=#

	# 4-State kinematic model
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 1] == states_s[i, 1] + dt * state_s_dot[i])
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 2] == states_s[i, 2] + 
				  dt * states_s[i, 4] * sin(states_s[i, 3] + beta[i]))
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 3] == states_s[i, 3] + 
				  dt * (states_s[i, 4] / l_rear * 
				  sin(beta[i]) - state_s_dot[i] * curvature[i]))
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 4] == states_s[i, 4] + dt * inputs[i, 1])


	current_input = optimizer.current_input

	# create cost function
	@NLexpression(optimizer.model, derivative_cost, 
				  sum{weights_derivative_states[j] * sum{states_s[i + 1, j] - 
				  states_s[i, j]^2, i = 1 : horizon}, j = 1 : 4} + 
				  sum{weights_derivative_inputs[j] * 
				  ((current_input[j] - inputs[1, j])^2 + 
				  sum{(inputs[i + 1, j] - inputs[i, j])^2, 
				  i = 1 : horizon - 1}), j = 1 : 2})
	@NLexpression(optimizer.model, control_cost, 0.5 * sum{weights_inputs[j] * 
				  (sum{(inputs[i, j] - reference_input[i, j])^2, 
				  i = 1 : horizon}), j = 1 : 2})
	@NLexpression(optimizer.model, path_cost, 0.5 * 
				  sum{weights_states[j] * sum{(states_s[i, j] - 
				  reference_s[i, j])^2, i = 1 : horizon + 1}, j = 1 : 4})

	optimizer.derivative_cost = derivative_cost
	optimizer.path_cost = path_cost
	optimizer.control_cost = control_cost

	optimizer.curvature = curvature
	optimizer.reference_s = reference_s

	@NLobjective(optimizer.model, Min, derivative_cost + path_cost + 
				 control_cost)
end

function init_path_following_regression!(optimizer::Optimizer, track::Track)
	# TODO: refactor init_path_following and init_learning_mpc

	# optimizer.weights_derivative_states = 0.1 * [0.0, 0.0, 0.1, 0.1, 0.1, 0.1] 
	# optimizer.weights_states = 1.0 * [0.0, 10.0, 1.0, 1.0, 1.0, 1.0]
	optimizer.weights_derivative_states = 0.1 * [0.0, 0.0, 0.1, 0.1, 0.1] 
	optimizer.weights_states = 1.0 * [0.0, 10.0, 1.0, 1.0, 1.0]

	weights_derivative_states = optimizer.weights_derivative_states
	weights_derivative_inputs = optimizer.weights_derivative_inputs
	weights_states = optimizer.weights_states
	weights_inputs = optimizer.weights_inputs

	dt = optimizer.agent.dt
	# println("optimiziation dt: ", dt)
	horizon = size(optimizer.agent.optimal_inputs)[1]
	l_front = optimizer.agent.l_front
	l_rear = optimizer.agent.l_rear
	
	iteration = optimizer.agent.current_iteration
	# initial_s = get_current_s(optimizer.agent)
	initial_s = [0.0; 0.0; 0.0; 1.0; 0.0]
	kappa = zeros(horizon)  # curvature
	
	optimizer.model = Model(solver=IpoptSolver(print_level=0, linear_solver="ma27",
											   max_cpu_time=0.09))

	@NLparameter(optimizer.model, reference_s[i = 1 : (horizon + 1), j = 1 : 5] == 0)
	reference_input = zeros(horizon, 2)

	state_s_lower_bound = [(- Inf) (- Inf) (- Inf) 0.0 0.0]
	state_s_upper_bound = [Inf Inf Inf Inf Inf]
	
	@variable(optimizer.model, state_s_lower_bound[j] <= 
			  				   optimizer.states_s[i = 1 : (horizon + 1), j = 1 : 5] <=
  			  				   state_s_upper_bound[j])
	@variable(optimizer.model, optimizer.agent.input_lower_bound[j] <= 
			  optimizer.inputs[i = 1 : horizon, j = 1 : 2] <= 
			  optimizer.agent.input_upper_bound[j])

	states_s = optimizer.states_s
	inputs = optimizer.inputs
	# println(getvalue(states_s[1]))
	
	@NLparameter(optimizer.model, optimizer.current_input[i = 1 : 2] == 0)
	@NLparameter(optimizer.model, optimizer.init_state_s[i = 1 : 5] == initial_s[i])
	# @NLparameter(optimizer.model, optimizer.init_state_s[i = 1 : 5] == 0)
	@NLconstraint(optimizer.model, [i = 1 : 5], 
				  states_s[1, i] == optimizer.init_state_s[i])

	# TODO: What's up with the track curvature in the optimization
	# get_curvature does not work in optimizaion. Using different
	# approach based on an estimate. Although exact curvature would be 
	# available
	# estimate curvature

	@NLparameter(optimizer.model, curvature[i = 1 : horizon] == kappa[i])

	# Create parameters for linear regression model
	@NLparameter(optimizer.model, optimizer.theta_vx[i = 1 : 3] == 0)
	@NLparameter(optimizer.model, optimizer.theta_vy[i = 1 : 4] == 0)
	@NLparameter(optimizer.model, optimizer.theta_psi_dot[i = 1 : 3] == 0)

	#=
	# Expressions for linear regression
	@NLexpression(optimizer.model, delta_vx[i = 1 : horizon], 
				  optimizer.theta_vx[1] * states_s[i, 6] * states_s[i, 4] +
				  optimizer.theta_vx[2] * states_s[i, 5] +
				  optimizer.theta_vx[3] * inputs[i, 1])
	@NLexpression(optimizer.model, delta_vy[i = 1 : horizon],
				  optimizer.theta_vy[1] * states_s[i, 6] / states_s[i, 5] + 
				  optimizer.theta_vy[2] * states_s[i, 5] * states_s[i, 4] + 
				  optimizer.theta_vy[3] * states_s[i, 4] / states_s[i, 5] + 
				  optimizer.theta_vy[4] * inputs[i, 2])
	@NLexpression(optimizer.model, delta_psi_dot[i = 1 : horizon],
				  optimizer.theta_psi_dot[1] * states_s[i, 4] / states_s[i, 5] + 
				  optimizer.theta_psi_dot[2] * states_s[i, 6] / states_s[i, 5] + 
				  optimizer.theta_psi_dot[3] * inputs[i, 2
	

	# Expression for s dot
	@NLexpression(optimizer.model, state_s_dot[i = 1 : horizon], 
				  (states_s[i, 5] * cos(states_s[i, 3]) - 
				   states_s[i, 6] * sin(states_s[i, 3])) / 
				  (1 - states_s[i, 2] * curvature[i]))
	=#

	@NLexpression(optimizer.model, beta[i = 1 : horizon], 
				  atan(l_rear / (l_front + l_rear) * tan(inputs[i, 2])))
	@NLexpression(optimizer.model, state_s_dot[i = 1 : horizon], 
				  (states_s[i, 4] * cos(states_s[i, 3]) - 
				   states_s[i, 5] * sin(states_s[i, 3])) / (1 - states_s[i, 2] * curvature[i]))

	# Model constraints
	# s
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 1] == states_s[i, 1] + dt * state_s_dot[i])
	# e_y
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 2] == states_s[i, 2] + 
									    dt * (states_s[i, 4] * sin(states_s[i, 3]) + 
									   		  states_s[i, 5] * cos(states_s[i, 3])))
	# e_psi
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 3] == states_s[i, 3] + 
									    dt * (states_s[i, 4] / l_front * 
									    sin(beta[i]) - state_s_dot[i] * curvature[i]))
	# v_x
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 4] == states_s[i, 4] + 
				  						# optimizer.theta_vx[2] * states_s[i, 4] + 
				  						optimizer.theta_vx[3] * inputs[i, 1])
	# @NLconstraint(optimizer.model, [i = 1 : horizon + 1], states_s[i, 4] >= 0.1)
	# v_y 
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 5] == states_s[i, 5] + 
				  						# optimizer.theta_vy[1] * states_s[i, 5] / states_s[i, 4] + 
				  						optimizer.theta_vy[4] * inputs[i, 2])

	#=
	# v_y 
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 6] == states_s[i, 6] + optimizer.theta_vy[1] * states_s[i, 6] / states_s[i, 5] + 
				  optimizer.theta_vy[2] * states_s[i, 5] * states_s[i, 4] + 
				  optimizer.theta_vy[3] * states_s[i, 4] / states_s[i, 5] + 
				  optimizer.theta_vy[4] * inputs[i, 2])
	# psi_dot
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 4] == states_s[i, 4] + optimizer.theta_psi_dot[1] * states_s[i, 4] / states_s[i, 5] + 
				  optimizer.theta_psi_dot[2] * states_s[i, 6] / states_s[i, 5] + 
				  optimizer.theta_psi_dot[3] * inputs[i, 2])
	# s
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 1] == states_s[i, 1] + dt * (states_s[i, 5] * cos(states_s[i, 3]) - 
										   states_s[i, 6] * sin(states_s[i, 3])) / 
										  (1 - states_s[i, 2] * curvature[i]))
	# e_y 
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 2] == states_s[i, 2] + dt * 
				  						(states_s[i, 5] * sin(states_s[i, 3]) +
				  						 states_s[i, 6] * cos(states_s[i, 3])))
	# e_psi
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 3] == states_s[i, 3] + dt * (states_s[i, 4] - 
				  					    curvature[i] * (states_s[i, 5] * cos(states_s[i, 3]) - 
									   states_s[i, 6] * sin(states_s[i, 3])) / 
									  (1 - states_s[i, 2] * curvature[i])))
	=#

	current_input = optimizer.current_input

	# create cost function
	@NLexpression(optimizer.model, derivative_cost, 
				  sum{weights_derivative_states[j] * sum{states_s[i + 1, j] - 
				  states_s[i, j]^2, i = 1 : horizon}, j = 1 : 5} + 
				  sum{weights_derivative_inputs[j] * 
				  ((current_input[j] - inputs[1, j])^2 + 
				  sum{(inputs[i + 1, j] - inputs[i, j])^2, 
				  i = 1 : horizon - 1}), j = 1 : 2})
	@NLexpression(optimizer.model, control_cost, 0.5 * sum{weights_inputs[j] * 
				  (sum{(inputs[i, j] - reference_input[i, j])^2, 
				  i = 1 : horizon}), j = 1 : 2})
	@NLexpression(optimizer.model, path_cost, 0.5 * 
				  sum{weights_states[j] * sum{(states_s[i, j] - 
				  reference_s[i, j])^2, i = 1 : horizon + 1}, j = 1 : 5})

	optimizer.derivative_cost = derivative_cost
	optimizer.path_cost = path_cost
	optimizer.control_cost = control_cost

	optimizer.curvature = curvature
	optimizer.reference_s = reference_s

	@NLobjective(optimizer.model, Min, derivative_cost + path_cost + 
				 control_cost)
end

function init_learning_mpc!(optimizer::Optimizer, track::Track)
	# TODO: Updated model from system identification

	optimizer.weights_derivative_states = 0.1 * [0.0, 0.0, 0.1, 0.1] 
	optimizer.weights_states = 1.0 * [0.0, 100.0, 0.1, 10.0]

	target_s = track.total_length
	max_e_y = track.width / 2
	# println("MAX EY: ", max_e_y)

	# v_max = optimizer.agent.v_max

	weights_derivative_states = optimizer.weights_derivative_states
	weights_derivative_inputs = optimizer.weights_derivative_inputs
	weights_states = optimizer.weights_states
	weights_inputs = optimizer.weights_inputs
	weights_lane = optimizer.weights_lane 
	weights_velocity = optimizer.weights_velocity
	# weights_obstacles = optimizer.weights_obstacles

	dt = optimizer.agent.dt
	horizon = size(optimizer.agent.optimal_inputs)[1]
	l_front = optimizer.agent.l_front
	l_rear = optimizer.agent.l_rear

	iteration = optimizer.agent.current_iteration
	initial_s = get_state_s(optimizer.agent, iteration)
	kappa = zeros(horizon)  # curvature
	num_selected_states = size(optimizer.agent.selected_states_s)[1]
	reference_input = zeros(horizon, 2)
	
	optimizer.model = Model(solver = IpoptSolver(print_level=0, linear_solver="ma27",
	 											 max_cpu_time=0.09))
	# optimizer.model = Model(solver = IpoptSolver(print_level=0, linear_solver="ma27"))

	# TODO: Bounds for velocity are wrong!
	@variable(optimizer.model, optimizer.agent.state_s_lower_bound[j] <= 
		  	  optimizer.states_s[i = 1 : (horizon + 1), j = 1 : 4] <=
		  	  optimizer.agent.state_s_upper_bound[j])
	@variable(optimizer.model, optimizer.agent.input_lower_bound[j] <= 
			  optimizer.inputs[i = 1 : horizon, j = 1 : 2] <= 
			  optimizer.agent.input_upper_bound[j])

	states_s = optimizer.states_s
	inputs = optimizer.inputs

	# @variable(optimizer.model, vel_soft[1 : (horizon + 1)] >= 0)
	@variable(optimizer.model, lane_soft[1 : (horizon + 1)] >= 0)
	@variable(optimizer.model, alpha[1 : num_selected_states] >= 0)

	@NLparameter(optimizer.model, optimizer.current_input[i = 1 : 2] == 0)
	@NLparameter(optimizer.model, optimizer.init_state_s[i = 1 : 4] == 
				 initial_s[i])
	@NLconstraint(optimizer.model, [i = 1 : 4], 
				  states_s[1, i] == optimizer.init_state_s[i])
	@NLparameter(optimizer.model, curvature[i = 1 : horizon] == kappa[i])
	@NLparameter(optimizer.model, selected_states[1 : num_selected_states, 
				 1 : 4] == 0)
	@NLparameter(optimizer.model, states_cost[1 : num_selected_states] == 0)
	@NLparameter(optimizer.model, optimizer.progress[1 : horizon + 1] == 0)
	@NLparameter(optimizer.model, weights_obstacles[1 : num_selected_states] == 0)
	@NLparameter(optimizer.model, weights_obstacle == 0)

	#=
	# No slack variables
	@NLconstraint(optimizer.model, [i = 2 : (horizon + 1)], states_s[i, 4] <=
				  v_max)
	@NLconstraint(optimizer.model, [i = 1 : (horizon + 1)], states_s[i, 2] <= 
				  max_e_y)
	@NLconstraint(optimizer.model, [i = 1 : (horizon + 1)], states_s[i, 2] >= 
				  - max_e_y)
	=#

	# @NLconstraint(optimizer.model, [i = 2 : (horizon + 1)], states_s[i, 4] <=
	#			  v_max + vel_soft[i])
	@NLconstraint(optimizer.model, [i = 2 : (horizon + 1)], states_s[i, 2] <= 
				  max_e_y + lane_soft[i])
	@NLconstraint(optimizer.model, [i = 2 : (horizon + 1)], states_s[i, 2] >= 
				  - max_e_y - lane_soft[i])

	@NLconstraint(optimizer.model, sum{alpha[i], 
				  i = 1 : num_selected_states} == 1)		
	for i = 1 : 4																										
		@NLconstraint(optimizer.model, states_s[horizon + 1, i] == 
				  	  sum{alpha[j] * selected_states[j, i],
				  	  j = 1 : num_selected_states})			
	end																																																																																																																																								

	@NLexpression(optimizer.model, beta[i = 1 : horizon], atan(l_rear / 
				  (l_front + l_rear) * tan(inputs[i, 2])))
	@NLexpression(optimizer.model, state_s_dot[i = 1 : horizon], 
				  states_s[i, 4] * cos(states_s[i, 3] + 
				  beta[i]) / (1 - states_s[i, 2] * curvature[i]))

	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 1] == states_s[i, 1] + dt * state_s_dot[i])
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 2] == states_s[i, 2] + 
				  dt * states_s[i, 4] * sin(states_s[i, 3] + beta[i]))
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 3] == states_s[i, 3] + 
				  dt * (states_s[i, 4] / l_rear * 
				  sin(beta[i]) - state_s_dot[i] * curvature[i]))
	@NLconstraint(optimizer.model, [i = 1 : horizon], 
				  states_s[i + 1, 4] == states_s[i, 4] + dt * inputs[i, 1])

	current_input = optimizer.current_input

	# create cost function
	@NLexpression(optimizer.model, derivative_cost, 
				  sum{weights_derivative_states[j] * sum{states_s[i + 1, j] - 
				  states_s[i, j]^2, i = 1 : horizon}, j = 1 : 4} + 
				  sum{weights_derivative_inputs[j] * 
				  ((current_input[j] - inputs[1, j])^2 + 
				  sum{(inputs[i + 1, j] - inputs[i, j])^2, 
				  i = 1 : horizon - 1}), j = 1 : 2})
	@NLexpression(optimizer.model, control_cost, 0.5 * sum{weights_inputs[j] * 
				  (sum{(inputs[i, j] - reference_input[i, j])^2, 
				  i = 1 : horizon}), j = 1 : 2})
	#=
	@NLexpression(optimizer.model, path_cost, 0.5 * 
				  sum{weights_states[j] * sum{(states_s[i, j] - 
				  reference_s[i, j])^2, i = 1 : horizon + 1}, j = 1 : 4})
	=#
	# TODO: use variables instead of magic numbers
	@NLexpression(optimizer.model, lane_cost, weights_lane * sum{10.0 * 
				  lane_soft[i] + 100.0 * lane_soft[i], i = 2 : (horizon + 1)})
	@NLexpression(optimizer.model, terminal_cost, sum{weights_obstacles[i] * 
				  alpha[i] * states_cost[i], i = 1 : num_selected_states})
	# @NLexpression(optimizer.model, velocity_cost, weights_velocity * sum{10.0 *
	#			  vel_soft[i] + 100 * vel_soft[i], i = 2 : (horizon + 1)})

	num_adv_agents = 0

	try 
		adv_agents = optimizer.adversarial_agents
		num_adv_agents = NUM_AGENTS - 1
		println("There are adversarial agents")
	catch
		num_adv_agents = 0
		println("There are no adversarial agents")
	end

	if NUM_AGENTS > 1
		iteration = agent.current_iteration
		agent = optimizer.agent
		radius_s = zeros(num_adv_agents)
		radius_e_y = zeros(num_adv_agents)
		
		@NLparameter(optimizer.model, 
					 optimizer.adv_states_s[1 : (horizon + 1), 1 : 4] == 0)

		# Asumming that both agents have the same size
		for i = 1 : num_adv_agents
			radius_s[i] = 2 * agent.l_front
			radius_e_y[i] = 2 * agent.width
		end

		# TODO: Consider rotation of the opponent?
		@NLexpression(optimizer.model, obstacle_cost, weights_obstacle * sum{- log(
					  ((states_s[j, 1] - optimizer.adv_states_s[j, 1]) / 
					  radius_s[1])^2 + ((states_s[j, 2] - 
					  optimizer.adv_states_s[j, 2]) / radius_e_y[1])^2 - 1),
					  j = 1 : (horizon + 1)})

		optimizer.obstacle_cost = obstacle_cost

		@NLexpression(optimizer.model, progress_cost, sum{optimizer.weights_progress[i] * 
					  optimizer.progress[i], i = 1 : horizon + 1})

		optimizer.progress_cost = progress_cost
		# println(optimizer.adv_states_s)
		# optimizer.adv_states_s = adv_states_s
	else
		println("Initialized without adverasarial agents")
	end

	optimizer.derivative_cost = derivative_cost
	# optimizer.path_cost = path_cost
	optimizer.control_cost = control_cost
	optimizer.lane_cost = lane_cost
	optimizer.terminal_cost = terminal_cost
	# optimizer.velocity_cost = velocity_cost

	optimizer.curvature = curvature
	# optimizer.reference_s = reference_s
	optimizer.selected_states = selected_states
	optimizer.states_cost = states_cost
	optimizer.weights_obstacle = weights_obstacle

	optimizer.alpha = alpha

	try 
		optimizer.weights_obstacles = weights_obstacles
		obstacle_cost = optimizer.obstacle_cost
		@NLobjective(optimizer.model, Min, derivative_cost +  # path_cost + 
					 control_cost + lane_cost + terminal_cost + velocity_cost +
					 obstacle_cost + progress_cost)
		# println("applying obstacle_cost")
	catch
		@NLobjective(optimizer.model, Min, derivative_cost +  # path_cost + 
					 control_cost + lane_cost + terminal_cost) # + velocity_cost)
	end
end

function init_lmpc_regression!(optimizer::Optimizer, track::Track)

	optimizer.weights_derivative_states = 0.1 * [0.0, 0.0, 0.1, 0.1, 0.1, 0.1] 
	optimizer.weights_states = 1.0 * [0.0, 100.0, 0.1, 0.1, 10.0, 10.0]

	target_s = track.total_length
	max_e_y = track.width / 2
	# v_max = optimizer.agent.v_max

	weights_derivative_states = optimizer.weights_derivative_states
	weights_derivative_inputs = optimizer.weights_derivative_inputs
	weights_states = optimizer.weights_states
	weights_inputs = optimizer.weights_inputs
	weights_lane = optimizer.weights_lane 
	weights_velocity = optimizer.weights_velocity
	# weights_obstacles = optimizer.weights_obstacles

	dt = optimizer.agent.dt
	horizon = size(optimizer.agent.optimal_inputs)[1]
	l_front = optimizer.agent.l_front
	l_rear = optimizer.agent.l_rear

	iteration = optimizer.agent.current_iteration
	initial_s = get_state_s(optimizer.agent, iteration)
	kappa = zeros(horizon)  # curvature
	num_selected_states = size(optimizer.agent.selected_states_s)[1]
	reference_input = zeros(horizon, 2)
	
	optimizer.model = Model(solver = IpoptSolver(print_level=0, linear_solver="ma27",
	 											 max_cpu_time=0.09))
	# optimizer.model = Model(solver = IpoptSolver(print_level=0, linear_solver="ma27"))

	# TODO: Bounds for velocity are wrong!
	@variable(optimizer.model, optimizer.agent.state_s_lower_bound[j] <= 
		  	  optimizer.states_s[i = 1 : (horizon + 1), j = 1 : 6] <=
		  	  optimizer.agent.state_s_upper_bound[j])
	@variable(optimizer.model, optimizer.agent.input_lower_bound[j] <= 
			  optimizer.inputs[i = 1 : horizon, j = 1 : 2] <= 
			  optimizer.agent.input_upper_bound[j])

	states_s = optimizer.states_s
	inputs = optimizer.inputs

	# @variable(optimizer.model, vel_soft[1 : (horizon + 1)] >= 0)
	@variable(optimizer.model, lane_soft[1 : (horizon + 1)] >= 0)
	@variable(optimizer.model, alpha[1 : num_selected_states] >= 0)

	@NLparameter(optimizer.model, optimizer.current_input[i = 1 : 2] == 0)
	@NLparameter(optimizer.model, optimizer.init_state_s[i = 1 : 6] == 
				 initial_s[i])
	@NLconstraint(optimizer.model, [i = 1 : 6], 
				  states_s[1, i] == optimizer.init_state_s[i])
	@NLparameter(optimizer.model, curvature[i = 1 : horizon] == kappa[i])
	@NLparameter(optimizer.model, selected_states[1 : num_selected_states, 
				 1 : 6] == 0)
	@NLparameter(optimizer.model, states_cost[1 : num_selected_states] == 0)
	@NLparameter(optimizer.model, optimizer.progress[1 : horizon + 1] == 0)
	@NLparameter(optimizer.model, weights_obstacles[1 : num_selected_states] == 0)
	@NLparameter(optimizer.model, weights_obstacle == 0)

	# @NLconstraint(optimizer.model, [i = 2 : (horizon + 1)], states_s[i, 5] <=
	#			  v_max + vel_soft[i])
	@NLconstraint(optimizer.model, [i = 2 : (horizon + 1)], states_s[i, 2] <= 
				  max_e_y + lane_soft[i])
	@NLconstraint(optimizer.model, [i = 2 : (horizon + 1)], states_s[i, 2] >= 
				  - max_e_y - lane_soft[i])

	@NLconstraint(optimizer.model, sum{alpha[i], 
				  i = 1 : num_selected_states} == 1)		
	for i = 1 : 6																										
		@NLconstraint(optimizer.model, states_s[horizon + 1, i] == 
				  	  sum{alpha[j] * selected_states[j, i],
				  	  j = 1 : num_selected_states})			
	end	

	# Create parameters for linear regression model
	@NLparameter(optimizer.model, optimizer.theta_vx[i = 1 : 3] == 0)
	@NLparameter(optimizer.model, optimizer.theta_vy[i = 1 : 4] == 0)
	@NLparameter(optimizer.model, optimizer.theta_psi_dot[i = 1 : 3] == 0)

	# Expressions for linear regression
	@NLexpression(optimizer.model, delta_vx[i = 1 : horizon], 
				  optimizer.theta_vx[1] * states_s[i, 6] * states_s[i, 4] +
				  optimizer.theta_vx[2] * states_s[i, 5] +
				  optimizer.theta_vx[3] * inputs[i, 1])
	@NLexpression(optimizer.model, delta_vy[i = 1 : horizon],
				  optimizer.theta_vy[1] * states_s[i, 6] / states_s[i, 5] + 
				  optimizer.theta_vy[2] * states_s[i, 5] * states_s[i, 4] + 
				  optimizer.theta_vy[3] * states_s[i, 4] / states_s[i, 5] + 
				  optimizer.theta_vy[4] * inputs[i, 2])
	@NLexpression(optimizer.model, delta_psi_dot[i = 1 : horizon],
				  optimizer.theta_psi_dot[1] * states_s[i, 4] / states_s[i, 5] + 
				  optimizer.theta_psi_dot[2] * states_s[i, 6] / states_s[i, 5] + 
				  optimizer.theta_psi_dot[3] * inputs[i, 2])

	# Expression for s dot
	@NLexpression(optimizer.model, state_s_dot[i = 1 : horizon], 
				  (states_s[i, 5] * cos(states_s[i, 3]) - 
				   states_s[i, 6] * sin(states_s[i, 3])) / 
				  (1 - states_s[i, 2] * curvature[i]))

	# Model constraints
	# v_x
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 5] == states_s[i, 5] + delta_vx[i])
	# v_y 
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 6] == states_s[i, 6] + delta_vy[i])
	# psi_dot
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 4] == states_s[i, 4] + delta_psi_dot[i])
	# s
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 1] == states_s[i, 1] + dt * state_s_dot[i])
	# e_y 
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 2] == states_s[i, 2] + dt * 
				  						(states_s[i, 5] * sin(states_s[i, 3]) +
				  						 states_s[i, 6] * cos(states_s[i, 3])))
	# e_psi
	@NLconstraint(optimizer.model, [i = 1 : horizon],
				  states_s[i + 1, 3] == states_s[i, 3] + dt * (states_s[i, 4] - 
				  					    curvature[i] * state_s_dot[i]))
	
	current_input = optimizer.current_input

	# create cost function
	@NLexpression(optimizer.model, derivative_cost, 
				  sum{weights_derivative_states[j] * sum{states_s[i + 1, j] - 
				  states_s[i, j]^2, i = 1 : horizon}, j = 1 : 6} + 
				  sum{weights_derivative_inputs[j] * 
				  ((current_input[j] - inputs[1, j])^2 + 
				  sum{(inputs[i + 1, j] - inputs[i, j])^2, 
				  i = 1 : horizon - 1}), j = 1 : 2})
	@NLexpression(optimizer.model, control_cost, 0.5 * sum{weights_inputs[j] * 
				  (sum{(inputs[i, j] - reference_input[i, j])^2, 
				  i = 1 : horizon}), j = 1 : 2})
	
	# TODO: use variables instead of magic numbers
	@NLexpression(optimizer.model, lane_cost, weights_lane * sum{10.0 * 
				  lane_soft[i] + 100.0 * lane_soft[i], i = 2 : (horizon + 1)})
	@NLexpression(optimizer.model, terminal_cost, sum{weights_obstacles[i] * 
				  alpha[i] * states_cost[i], i = 1 : num_selected_states})
	# @NLexpression(optimizer.model, velocity_cost, weights_velocity * sum{10.0 *
	#			  vel_soft[i] + 100 * vel_soft[i], i = 2 : (horizon + 1)})

	num_adv_agents = 0

	try 
		adv_agents = optimizer.adversarial_agents
		num_adv_agents = NUM_AGENTS - 1
		println("There are adversarial agents")
	catch
		num_adv_agents = 0
		println("There are no adversarial agents")
	end

	if NUM_AGENTS > 1
		iteration = agent.current_iteration
		agent = optimizer.agent
		radius_s = zeros(num_adv_agents)
		radius_e_y = zeros(num_adv_agents)
		
		@NLparameter(optimizer.model, 
					 optimizer.adv_states_s[1 : (horizon + 1), 1 : 6] == 0)

		# Asumming that both agents have the same size
		for i = 1 : num_adv_agents
			radius_s[i] = 2 * agent.l_front
			radius_e_y[i] = 2 * agent.width
		end

		# TODO: Consider rotation of the opponent?
		@NLexpression(optimizer.model, obstacle_cost, weights_obstacle * sum{- log(
					  ((states_s[j, 1] - optimizer.adv_states_s[j, 1]) / 
					  radius_s[1])^2 + ((states_s[j, 2] - 
					  optimizer.adv_states_s[j, 2]) / radius_e_y[1])^2 - 1),
					  j = 1 : (horizon + 1)})

		optimizer.obstacle_cost = obstacle_cost

		@NLexpression(optimizer.model, progress_cost, sum{optimizer.weights_progress[i] * 
					  optimizer.progress[i], i = 1 : horizon + 1})

		optimizer.progress_cost = progress_cost
		# println(optimizer.adv_states_s)
		# optimizer.adv_states_s = adv_states_s
	else
		println("Initialized without adverasarial agents")
	end

	optimizer.derivative_cost = derivative_cost
	# optimizer.path_cost = path_cost
	optimizer.control_cost = control_cost
	optimizer.lane_cost = lane_cost
	optimizer.terminal_cost = terminal_cost
	# optimizer.velocity_cost = velocity_cost

	optimizer.curvature = curvature
	# optimizer.reference_s = reference_s
	optimizer.selected_states = selected_states
	optimizer.states_cost = states_cost
	optimizer.weights_obstacle = weights_obstacle

	optimizer.alpha = alpha

	try 
		optimizer.weights_obstacles = weights_obstacles
		obstacle_cost = optimizer.obstacle_cost
		@NLobjective(optimizer.model, Min, derivative_cost +  # path_cost + 
					 control_cost + lane_cost + terminal_cost + velocity_cost +
					 obstacle_cost + progress_cost)
		# println("applying obstacle_cost")
	catch
		@NLobjective(optimizer.model, Min, derivative_cost +  # path_cost + 
					 control_cost + lane_cost + terminal_cost) # + velocity_cost)
	end

end

function solve_path_following!(optimizer::Optimizer, track::Track, 
							   reference::Array{Float64})
	# TODO: refactor solve_path_following and solve_learning_mpc!
	agent = optimizer.agent
	# println("STATE INITIALIZED: ", agent.state_initialized)
	iteration = agent.current_iteration
	horizon = size(optimizer.solution_inputs)[1]
	kappa = zeros(horizon)
	dt = optimizer.agent.dt

	for i = 1 : horizon - 1
		kappa[i] = get_curvature(track, 
								 optimizer.agent.predicted_s[i + 1, 1])
	end
	kappa[end] = get_curvature(track, optimizer.agent.predicted_s[end, 1] +
				 			   dt * optimizer.agent.predicted_s[end, 4])
	# println("kappa: ", kappa)
	init_state = zeros(4)
	init_state[1 : 3] = agent.states_s[iteration, 1 : 3]
	init_state[4] = sqrt(agent.states_s[iteration, 5]^2 + agent.states_s[iteration, 6]^2)
	println("INITIAL STATE: ", init_state)

	setvalue(optimizer.init_state_s, init_state)
	setvalue(optimizer.current_input, agent.inputs[iteration, :]')
	setvalue(optimizer.curvature, kappa)
	setvalue(optimizer.reference_s, ones(horizon + 1, 1) .* reference)

	solver_status = solve(optimizer.model)

	optimizer.solution_inputs = getvalue(optimizer.inputs)
	optimizer.solution_states_s = getvalue(optimizer.states_s)
	# optimizer.solution_inputs = repmat([input_upper_bound 0.0], horizon)

	publish_prediction(optimizer)
end

function pacejka(a)
    B = 1.0             # This value determines the steepness of the curve
    C = 1.25
    mu = 0.8            # Friction coefficient (responsible for maximum lateral tire force)
    m = 1.98
    g = 9.81
    D = mu * m * g/2
    C_alpha_f = D*sin(C*atan(B*a))
    return C_alpha_f
end

function solve_path_following_regression!(optimizer::Optimizer, track::Track, 
							   			  reference::Array{Float64})
	# TODO: refactor solve_path_following and solve_learning_mpc!
	agent = optimizer.agent
	# println("STATE INITIALIZED: ", agent.state_initialized)
	iteration = agent.current_iteration
	horizon = size(optimizer.solution_inputs)[1]
	kappa = zeros(horizon)
	dt = optimizer.agent.dt

	for i = 1 : horizon - 1
		kappa[i] = get_curvature(track, 
								 optimizer.agent.predicted_s[i + 1, 1])
	end
	if size(optimizer.agent.predicted_s, 2) == 6
		kappa[end] = get_curvature(track, optimizer.agent.predicted_s[end, 1] +
					 			   dt * optimizer.agent.predicted_s[end, 5])
	else
		kappa[end] = get_curvature(track, optimizer.agent.predicted_s[end, 1] +
					 			   dt * optimizer.agent.predicted_s[end, 4])
	end
	# println("kappa: ", kappa)

	# x : z[1]
    # y : z[2]
    # v_x : z[3]
    # v_y : z[4]
    # psi : z[5]
    # psiDot : z[6]
    # a : z[7]
    # d_f : z[8]

    l_rear = agent.l_rear
    l_front = agent.l_front
    current_state_s = agent.states_s[iteration, :]
    s, e_y, e_psi, psi_dot, v_x, v_y = current_state_s

	# Determine tire forces
	a_F = 0.0
    a_R = 0.0
    if abs(v_x) > 0.2
        a_F = atan((v_y + l_front * psi_dot) / abs(v_x)) - agent.current_input[2]
        a_R = atan((v_y - l_rear * psi_dot) / abs(v_x))
    end

    FyF = -pacejka(a_F)
    FyR = -pacejka(a_R)

	# agent.theta_vx = [-0.05 0.0 0.1]'
	# agent.theta_vy = [-0.05 -0.01 0.0 -0.05]'
	# agent.theta_psi_dot = [0.0 1.0 0.2]'
	# Set parameters for dynamic model
	# setvalue(optimizer.theta_vx, [0.1 0.0 0.1]')
	# setvalue(optimizer.theta_vy, [-0.4 0.0 0.0 0.0]')
	# setvalue(optimizer.theta_psi_dot, [-0.5 0.5 2.0]')
	# setvalue(optimizer.theta_vx, agent.theta_vx)
	# setvalue(optimizer.theta_vy, agent.theta_vy)
	# setvalue(optimizer.theta_psi_dot, agent.theta_psi_dot)

	# init_state = zeros(5)
	# init_state[1 : 3] = agent.states_s[iteration, 1 : 3]
	# init_state[4 : 5] = agent.states_s[iteration, 5 : 6]
	## init_state[4] = sqrt(agent.states_s[iteration, 5]^2 + agent.states_s[iteration, 6]^2)
	# println("INITIAL STATE: ", init_state)
	# setvalue(optimizer.init_state_s, init_state)

	println("INITIAL STATE: ", agent.states_s[iteration, :])
	setvalue(optimizer.init_state_s, current_state_s')

	println("CURRENT INPUT: ", agent.inputs[max(iteration - 1, 1), :])
	setvalue(optimizer.current_input, agent.inputs[max(iteration - 1, 1), :]')
	setvalue(optimizer.curvature, kappa)
	setvalue(optimizer.reference_s, ones(horizon + 1, 1) .* reference)

	solver_status = solve(optimizer.model)

	optimizer.solution_inputs = getvalue(optimizer.inputs)
	optimizer.solution_states_s = getvalue(optimizer.states_s)

	# if solver_status == :Infeasible && iteration > 10
	# 	exit()
	# end
	
	# optimizer.solution_inputs = repmat([input_upper_bound 0.0], horizon)

	# publish_prediction(optimizer)
end


function solve_learning_mpc!(optimizer::Optimizer, track::Track)
	# determines the optimal inputs and also states in s 
	agent = optimizer.agent
	iteration = agent.current_iteration
	horizon = size(optimizer.solution_inputs)[1]
	kappa = zeros(horizon)
	dt = optimizer.agent.dt

	for i = 1 : horizon - 1
		kappa[i] = get_curvature(track, 
								 optimizer.agent.predicted_s[i + 1, 1])
	end
	kappa[end] = get_curvature(track, optimizer.agent.predicted_s[end, 1] +
				 			   dt * optimizer.agent.predicted_s[end, 4])

	init_state = zeros(4)
	init_state[1 : 3] = agent.states_s[iteration, 1 : 3]
	init_state[4] = sqrt(agent.states_s[iteration, 5]^2 + agent.states_s[iteration, 6]^2)
	println("INITIAL STATE: ", init_state)

	setvalue(optimizer.init_state_s, init_state)
	
	# setvalue(optimizer.init_state_s, agent.states_s[iteration, :]')
	setvalue(optimizer.current_input, agent.inputs[iteration, :]')
	setvalue(optimizer.curvature, kappa)
	setvalue(optimizer.states_cost, agent.selected_states_cost)

	selected_states = zeros(NUM_CONSIDERED_STATES, 4)
	selected_states[:, 1 : 3] = agent.selected_states_s[:, 1 : 3]
	selected_states[:, 4] = sqrt(agent.selected_states_s[:, 5].^2 + 
								 agent.selected_states_s[:, 6].^2)
	setvalue(optimizer.selected_states, selected_states)

	setvalue(optimizer.weights_obstacles, agent.weights_states)
 	
	if NUM_AGENTS > 1
		propagate_adv_prediction!(optimizer)
		setvalue(optimizer.adv_states_s, optimizer.adv_predictions_s)
		distance = get_total_distance(optimizer, track.total_length)
		setvalue(optimizer.progress, vec(distance))
	end

	solver_status = solve(optimizer.model)

	optimizer.solution_inputs = getvalue(optimizer.inputs)
	optimizer.solution_states_s = getvalue(optimizer.states_s)

	# Testing maximum acceleration and straight steering
	# input_upper_bound = optimizer.agent.input_upper_bound[1]
	# optimizer.solution_inputs = repmat([input_upper_bound 0.0], horizon)

	#=
	if (solver_status == :Infeasible || solver_status == :UserLimit) && MODE == "learning"
		input_lower_bound = optimizer.agent.input_lower_bound[1]
		optimizer.solution_inputs = repmat([input_lower_bound 0.0], horizon)
		# optimizer.solution_states_s = simulate_s_kin(optimizer.agent, track, 
		#  								   			 optimizer.solution_inputs)
	else 
		optimizer.solution_inputs = getvalue(optimizer.inputs)
	end

	optimizer.solution_states_s = getvalue(optimizer.states_s)
	=#

	alpha = getvalue(optimizer.alpha)

	publish_prediction(optimizer)
end

function solve_learning_mpc!(optimizer::Optimizer, track::Track, leading::Bool)
	if leading 
		obstacle_weight = 1.0 * 0.01
	else 
		obstacle_weight = 0.1
	end

	setvalue(optimizer.weights_obstacle, obstacle_weight)

	solve_learning_mpc!(optimizer, track)
end

function solve_lmpc_regression!(optimizer::Optimizer, track::Track)
	# determines the optimal inputs and also states in s 
	agent = optimizer.agent
	iteration = agent.current_iteration
	horizon = size(optimizer.solution_inputs)[1]
	kappa = zeros(horizon)
	dt = optimizer.agent.dt

	for i = 1 : horizon - 1
		kappa[i] = get_curvature(track, 
								 optimizer.agent.predicted_s[i + 1, 1])
	end
	kappa[end] = get_curvature(track, optimizer.agent.predicted_s[end, 1] +
				 			   dt * optimizer.agent.predicted_s[end, 5])

	# Set parameters for dynamic model
	setvalue(optimizer.theta_vx, agent.theta_vx)
	setvalue(optimizer.theta_vy, agent.theta_vy)
	setvalue(optimizer.theta_psi_dot, agent.theta_psi_dot)

	println("INITIAL STATE: ", agent.states_s[iteration, :])
	
	setvalue(optimizer.init_state_s, agent.states_s[iteration, :]')
	setvalue(optimizer.current_input, agent.inputs[iteration, :]')
	setvalue(optimizer.curvature, kappa)
	setvalue(optimizer.states_cost, agent.selected_states_cost)
	setvalue(optimizer.selected_states, agent.selected_states_s)

	# setvalue(optimizer.weights_obstacles, agent.weights_states)
 	
	if NUM_AGENTS > 1
		propagate_adv_prediction!(optimizer)
		setvalue(optimizer.adv_states_s, optimizer.adv_predictions_s)
		distance = get_total_distance(optimizer, track.total_length)
		setvalue(optimizer.progress, vec(distance))
	end

	solver_status = solve(optimizer.model)

	optimizer.solution_inputs = getvalue(optimizer.inputs)
	optimizer.solution_states_s = getvalue(optimizer.states_s)

	# Testing maximum acceleration and straight steering
	# input_upper_bound = optimizer.agent.input_upper_bound[1]
	# optimizer.solution_inputs = repmat([input_upper_bound 0.0], horizon)

	#=
	if (solver_status == :Infeasible || solver_status == :UserLimit) && MODE == "learning"
		input_lower_bound = optimizer.agent.input_lower_bound[1]
		optimizer.solution_inputs = repmat([input_lower_bound 0.0], horizon)
		# optimizer.solution_states_s = simulate_s_kin(optimizer.agent, track, 
		#  								   			 optimizer.solution_inputs)
	else 
		optimizer.solution_inputs = getvalue(optimizer.inputs)
	end

	optimizer.solution_states_s = getvalue(optimizer.states_s)
	=#

	alpha = getvalue(optimizer.alpha)

	publish_prediction(optimizer)
end

function solve_lmpc_regression!(optimizer::Optimizer, track::Track, leading::Bool)
	if leading 
		obstacle_weight = 1.0 * 0.01
	else 
		obstacle_weight = 0.1
	end

	setvalue(optimizer.weights_obstacle, obstacle_weight)

	solve_lmpc_regression!(optimizer, track)
end


function warm_start_solution(optimizer::Optimizer, track_length)
	shifted_solution = optimizer.agent.predicted_s
	shifted_solution[:, 1] = optimizer.agent.predicted_s[:, 1] - track_length
	# shifted_solution[:, 2] = zeros(shifted_solution[:, 2])
	shifted_solution[1, 1] = 0.0
	setvalue(optimizer.states_s, shifted_solution)
	setvalue(optimizer.inputs, getvalue(optimizer.inputs))
end

function get_total_distance(optimizer::Optimizer, track_length::Int64)
	# calculate the distance on the track
	num_adv_agents = NUM_AGENTS - 1
	@assert num_adv_agents >= 1	

	horizon = size(optimizer.agent.predicted_s)[1] - 1
	current_laps = zeros(num_adv_agents + 1)
	s_coords = zeros(num_adv_agents + 1, horizon + 1)
	progress_on_track = zeros(num_adv_agents + 1, horizon + 1)

	for j = 1 : horizon + 1
		s_coords[1, j] = optimizer.agent.predicted_s[j, 1]
		current_laps[1] = optimizer.agent.current_lap
		progress_on_track[1, j] = (current_laps[1] - 1) * track_length + s_coords[1, j]

		for i = 1 : num_adv_agents
			s_coords[i + 1, j] = optimizer.adv_predictions_s[j, 1]  
			current_laps[i + 1] = optimizer.adv_current_lap
			progress_on_track[i + 1, j] = (current_laps[i + 1] - 1) * track_length + s_coords[i + 1, j]
		end 
	end

	# println("Progress on track: ", progress_on_track)
	# println(size(progress_on_track))

	return sum(progress_on_track .* (repmat([1; - 1]', horizon + 1))', 1)
end

function adjust_convex_hull(agent::Agent, optimizer::Optimizer)

    selected_states_s = agent.selected_states_s
    num_selected_states = size(selected_states_s)[1]
    horizon = size(optimizer.adv_predictions_s)[1] - 1

    dummy_weights = ones(size(selected_states_s)[1])
    for i = 1 : NUM_AGENTS - 1
        adv_predictions_s = optimizer.adv_predictions_s
        r_s = 2 * agent.l_front
        r_e_y = 2 * agent.width / 2
        for j = 1 : num_selected_states
            evaluate_ellipse = ((selected_states_s[j, 1] - 
                                adv_predictions_s[end, 1]) / r_s).^2 + 
                               ((selected_states_s[j, 2] - 
                                adv_predictions_s[end, 2]) / r_e_y).^2  
            if evaluate_ellipse <= 1
                agent.weights_states[j] = 10
            end
        end
    end

    sorted_weights = sortperm(agent.weights_states)
    max_weight = findmax(agent.weights_states[sorted_weights])

    if max_weight[1] > 1
        adjusted_weights_indeces = max_weight[2] : num_selected_states
        # 5 * 2 * horizon == num_selected_states
        for i = 1 : 2 * horizon : num_selected_states
            sum_weights = sum(agent.weights_states[i : i + 2 * horizon - 1])
            if sum_weights > 2 * horizon  # we know there are adjusted states
                max_index = findmax(agent.weights_states[i : i + 2 * horizon - 1])[2]
                # println(max_index)
                agent.weights_states[i + max_index : i + 2 * horizon - 1] = 
                    10 * ones(agent.weights_states[i + max_index : i + 2 * horizon - 1])
            end 
        end
    end
end


function propagate_adv_prediction!(optimizer::Optimizer, agent::Agent)
	# TODO: Change this using alpha!!
	try
		for i = 1 : NUM_AGENTS - 1
			optimizer.adv_predictions_s[1 : end - 1, :] = optimizer.adv_predictions_s[2 : end, :]
			last_pred_s = optimizer.adv_predictions_s[end, :]
			optimizer.adv_predictions_s[end, :] = last_pred_s
							
			optimizer.adv_predictions_s[end, 1] += last_pred_s[end - 1] * agent.dt
		end
	end
end
