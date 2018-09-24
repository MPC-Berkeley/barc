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

include("config.jl")
include("track.jl")
include("transformations.jl")
include("agent.jl")
# include("estimator.jl")
include("optimizer.jl")
include("low_level_controller.jl")
include("mpc_test.jl")


function race(num_laps::Int64)
	# Create strings for the current date
	hour_minute_second = Libc.strftime("%H%M%S", time())
	month_day = Libc.strftime("%m%d", time()) 

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

    if get_param(node_name * "/sim") == "simulation"
    	xy_sub = Subscriber("real_val", pos_info, xy_callback, (agent, track), 
   						queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
    else 
   		xy_sub = Subscriber("pos_info", pos_info, xy_callback, (agent, track), 
    					queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
   	end
    
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
	if NUM_AGENTS == 1
		mpc_model_convhull = MpcModel_convhull(agent, track)
		println("Succesfully initialized convhull model.")
	elseif NUM_AGENTS == 2
		mpc_model_obstacle = MpcModel_obstacle(agent, track)
		println("Succesfully initialized obstacle model.")
	else
		error("Too many agents.")
	end

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
		select_states(agent, track)
		if NUM_AGENTS == 1
			solveMpcProblem_convhull(mpc_model_convhull, optimizer, agent, track)
		else
			solveMpcProblem_obstacle(mpc_model_obstacle, optimizer, agent, track)
		end

		# Warm start optimizer
		max_acc = agent.input_upper_bound[1]
		warm_start_input = repmat([max_acc 0.0], HORIZON)
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
		if LEARNING && i <= NUM_PF_LAPS
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
			if MODE == "learning"
				race_lap!(agent, optimizer, low_level_controller, track ,xy_pub, 
						  theta_pub, selection_pub, input_pub, mpc_model_convhull)
			elseif MODE == "racing" 
				race_lap!(agent, optimizer, low_level_controller, track ,xy_pub, 
						  theta_pub, selection_pub, input_pub, mpc_model_obstacle)
			end
		end
	end

	# Set motor to neutral on shutdown. Stopping the Barc.
    neutralize!(low_level_controller)

    if !isdir(ENV["HOME_DIR"] * "/barc/recordings")
    	mkdir(ENV["HOME_DIR"] * "/barc/recordings")
    end
	# Create a folder for every day, if it does not exist yet: 
	cd(ENV["HOME_DIR"] * "/barc/recordings")  # move into recordings folder
	if isdir(month_day) 
		cd(month_day)
	else
		println("Creating a new folder for today: ", month_day)
		mkdir(month_day)
	end

	node_name = (get_name())[2 : end]
	if MODE == "path_following"
		save_trajectories(agent, ENV["HOME_DIR"] * "/barc/recordings/" * month_day * "/" * 
								 hour_minute_second * "_" * node_name * 
								 "_path_following_" * INITIALIZATION_TYPE * ".jld")
	elseif MODE == "learning"
		save_trajectories(agent, ENV["HOME_DIR"] * "/barc/recordings/" * month_day * "/" * 
								 hour_minute_second * "_" * node_name * 
								 "_lmpc_" * INITIALIZATION_TYPE * ".jld")
	elseif MODE == "racing"
		save_trajectories(agent, ENV["HOME_DIR"] * "/barc/recordings/" * month_day * "/" * 
								 hour_minute_second * "_" * node_name * 
								 "_racing_" * INITIALIZATION_TYPE  * ".jld")
	else 
		error("MODE $(MODE) is not defined. Please choose one of the following: 
			   path_following, learning or racing.")
	end
end


function xy_callback(msg::pos_info, agent::Agent, track::Track)
	if !agent.blocked
		current_xy = zeros(6)
		current_xy[1] = msg.x
		current_xy[2] = msg.y
		current_xy[3] = msg.v_x
		current_xy[4] = msg.v_y
		current_xy[5] = msg.psi
		current_xy[5] = atan2(sin(msg.psi), cos(msg.psi)) 
		current_xy[6] = msg.psiDot
		set_current_xy!(agent, current_xy, track)
		agent.state_initialized = true

		current_time = to_sec(get_rostime())
	    agent.time_states = current_time
	end
end

function publish_inputs(optimizer::Optimizer, input_pub, agent::Agent)
	current_time = to_sec(get_rostime())
	optimizer.first_input.motor = optimizer.solution_inputs[1, 1]
	optimizer.first_input.servo = optimizer.solution_inputs[1, 2]
	# println(optimizer.solution_inputs)
    publish(input_pub, optimizer.first_input)
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
		lap_index = current_lap + num_loaded_laps
		agent.iterations_needed[lap_index] = agent.current_iteration - 1

		# Get ROS time. Don't set to zero, because needs to be comparable to 
		# the other agent.
		time_now = to_sec(get_rostime())
		agent.time_log[agent.counter[end]] = time_now

		current_iteration = agent.current_iteration

		println("Iteration: ", current_iteration)

		# Make sure state is initialized, if not give more time for callbacks
		while sumabs(agent.states_s[current_iteration, :]) < 1e-5
			rossleep(Duration(0.005))
		end

		if agent.states_s[current_iteration, 1] > track.total_length / 2 && current_lap == 1 && current_iteration < 20
			agent.states_s[current_iteration, 1] = - (track.total_length - agent.states_s[current_iteration, 1])
		end

		println("Velocity: ", agent.states_s[current_iteration, 5])

		publish_inputs(optimizer, input_pub, agent)
		# Convert and send the calculated steering commands
		pwm_converter!(low_level_controller, agent.current_input[1], 
		 			   agent.current_input[2])

		agent.blocked = true

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

		if LEARNING && agent.current_lap > NUM_PF_LAPS
			agent.all_predictions[agent.counter[end], :, :] = agent.predicted_s
		else
			agent.all_predictions[agent.counter[end], :, [1, 2, 3, 5]] = agent.predicted_s
		end

		if agent.counter[end] == 1
			agent.theta_vx = zeros(agent.theta_vx)
			agent.theta_vy = zeros(agent.theta_vy)
			agent.theta_psi_dot = zeros(agent.theta_psi_dot)
		end
		
		agent.t_vx_log[agent.counter[end], :] = agent.theta_vx
		agent.t_vy_log[agent.counter[end], :] = agent.theta_vy
		agent.t_psidot_log[agent.counter[end], :] = agent.theta_psi_dot

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
			println("UPDATED INDEX FOR LAP: ", lap_index)

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
	num_buffer = NUM_STATES_BUFFER

	# Reset the weights and the predicted states
	# num_considered_states = size(agent.selected_states_s)[1]
	agent.weights_states = ones(NUM_CONSIDERED_STATES)
	agent.predicted_xy = zeros(agent.predicted_xy)

	# println("adv predictions: ", optimizer.adv_predictions_s)

	if LEARNING
		# TODO: Figure out a different way to know how many agents there are
		#######################################################################
		if NUM_AGENTS > 1
			current_s = agent.states_s[agent.current_iteration, 1]
			adv_s = optimizer.adv_predictions_s[1, 1]
			# distance = abs((agent.current_lap * track.total_length + current_s) - 
			#		   (optimizer.adv_current_lap * track.total_length + adv_s))
			distance = abs(current_s - adv_s)
			if distance > track.total_length / 2.0
				distance = track.total_length - distance
			end
			println("DISTANCE: ", distance)
			# if get_total_distance(simulator.optimizers[i], simulator.track.total_length) >= 0
			# println("direct distance on track: ", get_leading_agent_on_track(simulator, i))
			leading = get_leading_agent_on_track(track, agent, optimizer)
			agent.leading = leading
			println("Leading " * string(agent.index) * ": ", leading)
		end

		publish_selection(selection_pub, agent)

		tic()
		if agent.current_lap <= NUM_PF_LAPS
			solveMpcProblem_pathFollow(mpc_model, optimizer, agent, track, 
									   CURRENT_REFERENCE)
			# solve_path_following!(optimizer, track, CURRENT_REFERENCE)
		else 
			if NUM_AGENTS == 1
				select_states(agent, track)
			# elseif distance > NUM_HORIZONS * HORIZON * track.ds || leading
			elseif distance > 8.0 * track.ds || leading
				chosen_lap = NUM_LOADED_LAPS
				select_states(agent,track)
			else
				adv_agent_e_y = optimizer.adv_predictions_s[end, 2]
				# adv_agent_e_y = adv_agent.predicted_s[end, 2]
				
				#=
				adv_agent_v = optimizer.adv_predictions_s[end, 5]
				if adv_agent_v > agent.states_s[current_iteration, 5]
					select_states(agent)
				else
					select_states_smartly(agent, adv_agent_e_y, track)
					adjust_convex_hull(agent, optimizer)
				end
				=#

				adv_s = optimizer.adv_predictions_s[[1, HORIZON + 1], 1]
 				select_states_smartly(agent, adv_agent_e_y, adv_s, track)
				adjust_convex_hull(agent, optimizer)
			end

			if NUM_AGENTS == 2
				solveMpcProblem_obstacle(mpc_model, optimizer, agent, track, leading)
			elseif NUM_AGENTS == 1
				# println("Current s: ", agent.states_s[agent.current_iteration, :])
				solveMpcProblem_convhull(mpc_model, optimizer, agent, track)
			end
		end
		toc()
	else
		# println("s now: ", agent.states_s[agent.current_iteration, :])
		# println("s previous: ", agent.states_s[max(agent.current_iteration - 1, 1), :])
		if agent.states_s[agent.current_iteration, :] == agent.states_s[max(agent.current_iteration - 1, 1), :]
			println("=========================Same state used!!!!=======================")
		end
		solveMpcProblem_pathFollow(mpc_model, optimizer, agent, track, 
								   CURRENT_REFERENCE)
		# solve_path_following!(optimizer, track, CURRENT_REFERENCE)
	end

	# Determines the optimal inputs and also states in s
	agent.optimal_inputs = optimizer.solution_inputs
	agent.predicted_s = optimizer.solution_states_s
	# println("predicted s: ", agent.predicted_s)

	# Convert the prediction from s-coordinates to xy-coordinates for plotting
	for j = 1 : HORIZON + 1
		agent.predicted_xy[j, :] = s_to_xy(track, agent.predicted_s[j, :])
	end

	publish_xy(xy_pub, agent)
	publish_theta(theta_pub, agent)

	# Apply the first input
	agent.inputs[current_iteration, :] = agent.optimal_inputs[1, :]
	agent.current_input = agent.optimal_inputs[1, :]
	# println("optimal inputs:", agent.optimal_inputs[:, :])
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


function get_leading_agent_on_track(track::Track, agent::Agent, optimizer::Optimizer)
	current_iteration = agent.current_iteration
	current_s = agent.states_s[current_iteration, 1]
	adv_s = optimizer.adv_predictions_s[1, 1]
	track_length = track.total_length

	#=
	if current_s <= track_length / 2
		if current_s > adv_s
			return true
		elseif current_s <= adv_s && adv_s <= current_s + track_length / 2
			return false
		else 
			return true
		end
	else
		if current_s < adv_s
			return false
		elseif adv_s <= current_s && adv_s > current_s - track_length / 2
			return true
		else
			return false
		end
	end
	=#

	distance = current_s - adv_s

	if distance > 0
		if distance < track.total_length / 2.0
			return true
		else 
			return false
		end
	else
		if - distance < track.total_length / 2.0
			return false
		else 
			return true
		end
	end

end	


# Start race() function
if !isinteractive()
    race(NUM_LAPS)
end