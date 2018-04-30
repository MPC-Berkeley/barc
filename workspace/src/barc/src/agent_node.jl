#!/usr/bin/env julia

#=
	File name: agent_node.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#


using RobotOS
@rosimport barc.msg: ECU, pos_info, Vel_est, prediction, xy_prediction, theta, selected_states
@rosimport sensor_msgs.msg : Imu
@rosimport marvelmind_nav.msg : hedge_pos
@rosimport std_msgs.msg : Header
rostypegen()

using barc.msg
using sensor_msgs.msg
using marvelmind_nav.msg
using std_msgs.msg

include("config_2.jl")
include("track.jl")
include("transformations.jl")
include("agent.jl")
# include("estimator.jl")
include("optimizer.jl")
include("low_level_controller.jl")
include("mpc_test.jl")


function race(num_laps::Int64)
	# Initialize ROS node"
	init_node("agent_node")
    # init_node("agent_node_" * string(agent.index))
    node_name = get_name()

	# Initialization
	init_states = INIT_STATES
	v_max = V_MAX[1]
	color = COLOR[1]

    track = Track()
    init!(track)

    agent = Agent()
	init!(agent, 1, track, MAXIMUM_NUM_ITERATIONS, HORIZON, NUM_LAPS, 
		  NUM_CONSIDERED_STATES, NUM_LOADED_LAPS, v_max, color)
	set_state_s!(agent, init_states[1, :], track)

    xy_pub = Publisher("xy_prediction", xy_prediction, 
    				   queue_size=1)::RobotOS.Publisher{barc.msg.xy_prediction}
    xy_sub = Subscriber("pos_info", pos_info, xy_callback, (agent, track), 
    					queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
    # xy_sub = Subscriber("real_val", pos_info, xy_callback, (agent, track), 
    #					queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
    theta_pub = Publisher("theta", theta, queue_size=1)::RobotOS.Publisher{barc.msg.theta}
    selection_pub = Publisher("selected_states", selected_states, queue_size=1)::RobotOS.Publisher{barc.msg.selected_states}

    #=
    estimator = StateEstimator()
    init!(estimator, agent.dt)
    =#

    #=
    identifier = SystemIdentifier()
    init!(identifier)
    =#

    optimizer = Optimizer()
	init!(optimizer, agent, HORIZON)

	mpc_model_pf = MpcModel_pF(agent, track)
	mpc_model_convhull = MpcModel_convhull(agent, track)
	println("Succesfully initialized convhull model.")

	# Create an Optimizer object for the first path following lap of the LMPC
	#= 
	optimizer_path_following = nothing
	if LEARNING
		optimizer_path_following = Optimizer()
		init!(optimizer_path_following, agent, HORIZON)
	end
	=#

    input_pub = Publisher("ecu", ECU, 
						  queue_size=1)::RobotOS.Publisher{barc.msg.ECU}

    low_level_controller = LowLevelController()
    init!(low_level_controller)

	if LEARNING
		#=
		init_path_following!(optimizer_path_following, track)

		if SYS_ID
			# Initialize all necessary optimizers in the beginning
			# init_lmpc_regression!(optimizer, track)
			init_path_following_regression!(optimizer, track)
		else 
			init_learning_mpc!(optimizer, track)
		end
		=#

		# Do the system identification once to speed up the whole process
		# identify_system!(agent)
		select_states(agent)
		solveMpcProblem_convhull(mpc_model_convhull, optimizer, agent, track)

		# Warm start optimizer
		max_acc = agent.input_upper_bound[1]
		warm_start_input = repmat([max_acc 0.0], HORIZON)

		#=
		if size(getvalue(optimizer.states_s), 2) == 6
			warm_start_states = zeros(agent.predicted_s)
			warm_start_states[:, 5] = cumsum([0.0; agent.dt * max_acc * ones(HORIZON)])
			# For the system identification the solution is warm started based 
			# on the speed of the path_following
			if SYS_ID
				# warm_start_states[:, 5] += CURRENT_REFERENCE[4]
				warm_start_states[:, 5] = CURRENT_REFERENCE[4]
			end
			warm_start_states[:, 1] = cumsum(agent.dt * warm_start_states[:, 5])
		elseif size(getvalue(optimizer.states_s), 2) == 4
			warm_start_states = zeros(size(agent.predicted_s, 1), 4)
			warm_start_states[:, 4] = cumsum([0.0; agent.dt * max_acc * ones(HORIZON)])
			if SYS_ID
				# warm_start_states[:, 5] += CURRENT_REFERENCE[4]
				warm_start_states[:, 4] = CURRENT_REFERENCE[4]
			end
			warm_start_states[:, 1] = cumsum(agent.dt * warm_start_states[:, 4])
		elseif size(getvalue(optimizer.states_s), 2) == 5
			warm_start_states = zeros(size(agent.predicted_s, 1), 5)
			if SYS_ID
				# warm_start_states[:, 5] += CURRENT_REFERENCE[4]
				warm_start_states[:, 4] = CURRENT_REFERENCE[4]
			end
			warm_start_states[:, 1] = cumsum(agent.dt * warm_start_states[:, 4])
			println("warm start states: ", warm_start_states)
		else
			println("WRONG SIZE OF STATES")
			exit()
		end

		# println("WARM START STATES: ", warm_start_states)
		# println("WARM START INPUTS: ", warm_start_input)
		setvalue(optimizer.states_s, warm_start_states)
		setvalue(optimizer.inputs, warm_start_input)
		=#
	else
		# init_path_following!(optimizer, track)
	end

	# Add states from the last loaded lap to the current lap
	current_lap = agent.current_lap

	println("Sleeping for ", get_param(node_name * "/time_offset"), " seconds.")
	rossleep(get_param(node_name * "/time_offset"))
	
	for i = 1 : num_laps
		println("Lap: ", i)

		# Do path following for the first lap, if LMPC is used.
		if LEARNING && i <= 1
			race_lap!(agent, optimizer, low_level_controller, 
					  track, xy_pub, theta_pub, selection_pub, input_pub, mpc_model_pf)
			# Make sure the same agent object is used after the first path 
			# following lap has been completed
			# optimizer.agent = optimizer_path_following.agent
			# setvalue(optimizer.inputs, getvalue(optimizer_path_following.inputs))
		elseif !LEARNING
			race_lap!(agent, optimizer, low_level_controller, 
					  track, xy_pub, theta_pub, selection_pub, input_pub, mpc_model_pf)
		else
			race_lap!(agent, optimizer, low_level_controller, track ,xy_pub, 
					  theta_pub, selection_pub, input_pub, mpc_model_convhull)
			#=
			if agent.current_iteration > 30
				filename = "/home/lukas/simulations/states_infeasible.jld"
				jldopen(filename, "w") do file
				    JLD.write(file, "trajectories_s", agent.states_s[1 : agent.current_iteration , :])
				    JLD.write(file, "previous_inputs", agent.inputs[1 : agent.current_iteration, :])
				end
				exit()
			end
			=#
		end
	end

	filename = "/home/lukas/simulations/theta.jld"
	jldopen(filename, "w") do file
	#    JLD.write(file, "vx", agent.t_vx_log[1 : agent.t_counter, :])
	#    JLD.write(file, "vy", agent.t_vy_log[1 : agent.t_counter, :])
	#    JLD.write(file, "psi_dot", agent.t_psi_log[1 : agent.t_counter, :])
		JLD.write(file, "vx", agent.t_vx_log)
	    JLD.write(file, "vy", agent.t_vy_log)
	    JLD.write(file, "psi_dot", agent.t_psi_log)
	end

	if MODE == "path_following"
		save_trajectories(agent, "/home/lukas/simulations/" * get_name() * 
								 "_path_following_" * INITIALIZATION_TYPE * ".jld")
		# save_trajectories(agent, "/home/lukas/simulations/lmpc.jld")
	elseif MODE == "learning"
		save_trajectories(agent, "/home/lukas/simulations/" * get_name() * 
								 "_lmpc_theta_test_" * INITIALIZATION_TYPE * ".jld")
		# save_trajectories(agent, "/home/lukas/simulations/lmpc_2.jld")
	end

    # Set motor to neutral on shutdown
    neutralize!(low_level_controller)
end


function xy_callback(msg::pos_info, agent::Agent, track::Track)
	if !agent.blocked
		current_xy = zeros(6)
		current_xy[1] = msg.x
		current_xy[2] = msg.y
		current_xy[3] = msg.v_x
		current_xy[4] = msg.v_y
		current_xy[5] = msg.psi
		if msg.psi < - pi 
			current_xy[5] -= min(ceil(msg.psi / (2 * pi)), - 1.0) * 2 * pi
		elseif msg.psi > pi
			current_xy[5] -= max(floor(msg.psi / (2 * pi)), 1.0) * 2 * pi
		end
		current_xy[6] = msg.psiDot
		set_current_xy!(agent, current_xy, track)
		agent.state_initialized = true

		current_time = to_sec(get_rostime())
	    # println("States time: ", current_time)
	    # println("States delta: ", current_time - agent.time_states)
	    agent.time_states = current_time
	end
end

function publish_inputs(optimizer::Optimizer, input_pub, agent::Agent)
	current_time = to_sec(get_rostime())
	optimizer.first_input.motor = optimizer.solution_inputs[1, 1]
	optimizer.first_input.servo = optimizer.solution_inputs[1, 2]
    publish(input_pub, optimizer.first_input)
    # println("Input time: ", current_time)
    # println("Input delta: ", current_time - agent.time_inputs)
    agent.time_inputs = current_time
end


function race_lap!(agent::Agent, optimizer::Optimizer, 
				   low_level_controller::LowLevelController, track::Track, 
				   xy_pub, theta_pub, selection_pub, input_pub, mpc_model)
	# set node rate
	loop_rate = Rate(1 / agent.dt)

	leading_index = 0
	following_index = 0

	iteration = 1
	finished = false

	prev_s = agent.states_s[1, 1]
	current_s = agent.states_s[1, 1]

	num_loaded_laps = NUM_LOADED_LAPS
	current_lap = agent.current_lap
	lap_index = current_lap + num_loaded_laps
	# num_considered_states = size(agent.selected_states_s)[1]
	num_buffer = NUM_STATES_BUFFER
	considered_indeces = 1 : num_buffer
	shifted_s = ([track.total_length 0 0 0 0 0] .* ones(num_buffer, 6))
	single_shifted_s = ([track.total_length 0 0 0 0 0] .* ones(1, 6))

	mapping = [5, 6, 4]

	while !is_shutdown() && iteration <= MAXIMUM_NUM_ITERATIONS && !finished
		current_iteration = agent.current_iteration

		println("Iteration: ", current_iteration)

		# Make sure state is initialized, if not give more time for callbacks
		while sumabs(agent.states_s[current_iteration, :]) < 1e-5
			rossleep(Duration(0.005))
		end

		publish_inputs(optimizer, input_pub, agent)
		# Convert and send the calculated steering commands
		pwm_converter!(low_level_controller, agent.current_input[1], 
		 			   agent.current_input[2])
		# Two step input delay for steering
		# pwm_converter!(low_level_controller, agent.current_input[1], 
		#			   agent.optimal_inputs[3, 2]) 

		agent.blocked = true

	    # println("DELTA TIMES: ", agent.time_states - agent.time_inputs)

		race_iteration!(agent, optimizer, low_level_controller, track, xy_pub, 
						theta_pub, selection_pub, mpc_model)

		# Constantly add the current state to the end of the trajectory of the 
		# previous lap
		if current_lap > 1 && current_iteration <= num_buffer
			previous_lap = agent.current_lap - 1 + NUM_LOADED_LAPS
			iterations_needed = agent.iterations_needed[previous_lap]
			if current_lap == 2 && NUM_LOADED_LAPS == 0
				last_index = iterations_needed + current_iteration
			else
				last_index = num_buffer + iterations_needed + current_iteration
			end
			# Because the next lap already starts at iteration 2
			if current_iteration == 2
				agent.trajectories_s[previous_lap, last_index - 1, :] = agent.states_s[1, :] + single_shifted_s
				agent.trajectories_xy[previous_lap, last_index - 1, :] = agent.states_xy[1, :]
				agent.previous_inputs[previous_lap, last_index - 1, :] = agent.inputs[1, :]
			end
			agent.trajectories_s[previous_lap, last_index, :] = agent.states_s[current_iteration, :] + single_shifted_s
			agent.trajectories_xy[previous_lap, last_index, :] = agent.states_xy[current_iteration, :]
			agent.previous_inputs[previous_lap, last_index, :] = agent.inputs[current_iteration, :]
		end

		# Only take the number of iterations minus the current 
		# iteration, because it is already part of the next lap.
		lap_index = current_lap + num_loaded_laps
		
		if lap_index > num_loaded_laps + 1 && current_iteration <= num_buffer
			# Add the last couple of states to the beginning of the next lap
			needed_iterations = agent.iterations_needed[lap_index - 1]
			iteration_index = needed_iterations + current_iteration
			if current_iteration == 2
				agent.trajectories_s[lap_index, 1, :] = squeeze(agent.trajectories_s[lap_index - 1, iteration_index - 1, :], 1) - single_shifted_s
				agent.trajectories_xy[lap_index, 1, :] = agent.trajectories_xy[lap_index - 1, iteration_index - 1, :]
				agent.previous_inputs[lap_index, 1, :] = agent.previous_inputs[lap_index - 1, iteration_index - 1, :]
			end
			agent.trajectories_s[lap_index, current_iteration, :] = squeeze(agent.trajectories_s[lap_index - 1, iteration_index, :], 1) - single_shifted_s
			agent.trajectories_xy[lap_index, current_iteration, :] = agent.trajectories_xy[lap_index - 1, iteration_index, :]
			agent.previous_inputs[lap_index, current_iteration, :] = agent.previous_inputs[lap_index - 1, iteration_index, :]
		end

		# TODO:
		#=
		#  Prevents restoration failure. 
		if agent.predicted_s[2, 1] > track.total_length
			warm_start_solution(optimizer, track.total_length)
		end
		=#

		agent_counter = agent.counter[agent.current_session]
		current_dynamics = [agent.states_s[current_iteration, mapping] agent.inputs[current_iteration, :]]
		agent.dynamics[agent.num_sessions, agent_counter, :] = current_dynamics
		agent.counter[agent.current_session] += 1

		current_s = agent.states_s[current_iteration, 1]

		# Check if there is a big change in the s-coordinate, which
		# indicates that the current lap has been terminated.
		if abs(prev_s - current_s) >= 0.5 * track.total_length && 
			agent.current_iteration > 1

			s_coords = agent.states_s[current_iteration, :]
			agent.states_s[current_iteration, 1] = get_s_coord(track, s_coords[1])

			final_s = agent.states_s[current_iteration, :]
			final_xy = agent.states_xy[current_iteration, :]
			final_input = agent.inputs[current_iteration, :]

			current_lap = agent.current_lap
			println("Current Lap: ", current_lap)
 
			lap_index = current_lap + num_loaded_laps
			agent.iterations_needed[lap_index] = agent.current_iteration - 1

			# Reset states and inputs
			agent.states_s = zeros(agent.states_s)
			agent.states_xy = zeros(agent.states_xy)
			agent.inputs = zeros(agent.inputs)

			# Initalize states and inputs for next lap
			agent.states_s[1, :] = final_s
			agent.states_xy[1, :] = final_xy
			agent.inputs[1, :] = final_input

			if lap_index + 1 <= NUM_LAPS + num_loaded_laps
				lap_index += 1
				shifted_iteration = 1 + num_buffer
				agent.trajectories_s[lap_index, shifted_iteration, :] = agent.states_s[1, :]
				agent.trajectories_xy[lap_index, shifted_iteration, :] = agent.states_xy[1, :]
				agent.previous_inputs[lap_index, shifted_iteration, :] = agent.inputs[1, :]
			end

			# Make sure there are more iterations in the lap than there saved 
			# states in the buffer
			if agent.current_iteration < NUM_STATES_BUFFER
				error("Too many states in the buffer. Reduce the buffer size.")
			end

			agent.current_lap += 1

			agent_counter = agent.counter[agent.current_session]
			current_dynamics = [agent.states_s[1, mapping] agent.inputs[1, :]]
			agent.dynamics[agent.num_sessions, agent_counter, :] = current_dynamics
			agent.counter[agent.current_session] += 1

			# Set iteration to 2, because the next lap has already been 
			# reached
			agent.current_iteration = 2
			finished = true
		else
			current_iteration = agent.current_iteration
			if current_lap == 1 && num_loaded_laps == 0
				# Do not shift if it is the overall first lap, because there 
				# is no data, which can be appended
				shifted_iteration = current_iteration
			else
				shifted_iteration = current_iteration + num_buffer
			end

			# Save actual trajectories and inputs.
			agent.trajectories_s[lap_index, shifted_iteration, :] = agent.states_s[current_iteration, :]
			agent.trajectories_xy[lap_index, shifted_iteration, :] = agent.states_xy[current_iteration, :]
			agent.previous_inputs[lap_index, shifted_iteration, :] = agent.inputs[current_iteration, :]
		end

		prev_s = agent.states_s[current_iteration, 1]

		if !finished
			agent.current_iteration += 1
			iteration += 1
		end

		agent.state_initialized = false
		agent.blocked = false
		# simulator.elapsed_time += simulator.agents[1].dt

		rossleep(loop_rate)
	end

	println("Finished lap")
end


function race_iteration!(agent::Agent, optimizer::Optimizer, 
						 low_level_controller::LowLevelController, track::Track, 
						 xy_pub, theta_pub, selection_pub, mpc_model)
	distance = nothing
	leading = false

	# println("I am solving agent $(agent.index)")

	current_iteration = agent.current_iteration

	# Reset the weights and the predicted states
	# num_considered_states = size(agent.selected_states_s)[1]
	num_buffer = NUM_STATES_BUFFER
	agent.weights_states = ones(num_buffer)
	agent.predicted_xy = zeros(agent.predicted_xy)

	if LEARNING
		# TODO: Figure out a different way to know how many agents there are
		#######################################################################
		if NUM_AGENTS > 1
			distance = get_distance_on_track(simulator)
			# println("DISTANCE: ", distance)
			# if get_total_distance(simulator.optimizers[i], simulator.track.total_length) >= 0
			# println("direct distance on track: ", get_leading_agent_on_track(simulator, i))
			if get_leading_agent_on_track(simulator, 1) >= 0
				leading = true
			end
		end

		publish_selection(selection_pub, agent)

		tic()
		if agent.current_lap <= 1
			solveMpcProblem_pathFollow(mpc_model, optimizer, agent, track, CURRENT_REFERENCE)
			# solve_path_following!(optimizer, track, CURRENT_REFERENCE)
		else 
			if NUM_AGENTS == 1
				select_states(agent)
			elseif distance > 2 * simulator.horizon * simulator.track.ds || leading
				chosen_lap = simulator.num_loaded_laps
				select_states(agent)
			else
				adv_agent = optimizer.adversarial_agents[1]
				adv_agent_iter = adv_agent.current_iteration
				adv_agent_e_y = adv_agent.states_s[adv_agent_iter, 2]
				# adv_agent_e_y = adv_agent.predicted_s[end, 2]
				adv_agent_v = adv_agent.states_s[adv_agent_iter, 4]
				if adv_agent_v > agent.states_s[current_iteration, 4]
					select_states(agent)
				else
					select_states_smartly(agent, adv_agent_e_y, track)
					adjust_convex_hull(optimizer)
				end
			end

			solveMpcProblem_convhull(mpc_model, optimizer, agent, track)

			#=
			if SYS_ID
				# solve_lmpc_regression!(optimizer, track)
				# ref = [0.0 0.0 0.0 0.0 1.0 0.0]
				ref = [0.0 0.0 0.0 1.0 0.0]
				solve_path_following_regression!(optimizer, track, ref)
			else 
				solve_learning_mpc!(optimizer, track, leading)
			end
			=#
		end
		toc()
	else
		#=
		if agent.current_lap > 1
			identify_system!(agent)
		end
		=#

		solveMpcProblem_pathFollow(mpc_model, optimizer, agent, track, CURRENT_REFERENCE)
		# solve_path_following!(optimizer, track, CURRENT_REFERENCE)
	end

	# Determines the optimal inputs and also states in s
	agent.optimal_inputs = optimizer.solution_inputs
	agent.predicted_s = optimizer.solution_states_s
	println("predicted s: ", agent.predicted_s)

	# Convert the prediction from s-coordinates to xy-coordinates for plotting
	for j = 1 : HORIZON + 1
		agent.predicted_xy[j, :] = s_to_xy(track, agent.predicted_s[j, :])
	end

	publish_xy(xy_pub, agent)
	publish_theta(theta_pub, agent)

	# Apply the first input
	agent.inputs[current_iteration, :] = agent.optimal_inputs[1, :]
	agent.current_input = agent.optimal_inputs[1, :]
	println("optimal inputs:", agent.optimal_inputs[:, :])
end

function publish_xy(pub_xy, agent)
	agent.xy_predictions.x = agent.predicted_xy[:, 1]
	agent.xy_predictions.y = agent.predicted_xy[:, 2]
	agent.xy_predictions.psi = agent.predicted_xy[:, 5]
	agent.xy_predictions.acc = agent.optimal_inputs[:, 1]
	agent.xy_predictions.steering = agent.optimal_inputs[:, 2]
	agent.xy_predictions.current_lap = agent.current_lap
	publish(pub_xy, agent.xy_predictions)
end

function publish_theta(pub_theta, agent)
	agent.thetas.theta_vx = agent.theta_vx
	agent.thetas.theta_vy = agent.theta_vy
	agent.thetas.theta_psi_dot = agent.theta_psi_dot
	publish(pub_theta, agent.thetas)
end

function publish_selection(selection_pub, agent)
	agent.selection.x = agent.selected_states_xy[:, 1]
	agent.selection.y = agent.selected_states_xy[:, 2]
	publish(selection_pub, agent.selection)
end

# Start race() function
if !isinteractive()
    race(NUM_LAPS)
end


function get_num_loaded_laps(filename::ASCIIString)
    Data = load(filename)
    return size(Data["trajectories_s"])[1]
end


function get_num_loaded_laps(filenames::Array{ASCIIString})
	num_loaded_laps = 0
	for filename in filenames
    	Data = load(filename)
    	num_loaded_laps += size(Data["trajectories_s"])[1]
    end

    return num_loaded_laps
end


function get_distance_on_track(agent::Agent, optimizer::Optimizer, track::Track)
	# calculate the distance on the track
	@assert NUM_AGENTS > 1
	track_length = track.total_length
	s_coords = zeros(NUM_AGENTS)

	iteration = agent.current_iteration
	# TODO: Assuming only two agents
	s_coords[1] = agent.states[iteration, 1]
	s_coords[2] = optimizer.adv_predictions[1, 1]

	distance = abs(sum(s_coords .* [1; - 1]))

	if distance > track_length / 2
		distance = track_length - distance
	end

	return distance
end


function get_leading_agent_on_track(track::Track, idx::Int64)
	# TODO: fix this
	track_length = track.total_length
	s_coords = zeros(NUM_AGENTS)

	if idx == 1
		increment = 1
		end_idx = 2
	elseif idx == 2
		increment = - 1
		end_idx = 1
	end

	counter = 1
	for i = idx : increment : end_idx
		iteration = simulator.agents[i].current_iteration
		s_coords[counter] = simulator.agents[i].states_s[iteration, 1]
		counter += 1
	end

	distance = sum(s_coords .* [1; - 1])

	return distance
end