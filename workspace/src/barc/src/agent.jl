#!/usr/bin/env julia

#=
	File name: agent.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

using Distances

include("track.jl")
include("transformations.jl")
include("filesystem_helpers.jl")


type Agent
	# index to identify the agent
	index::Int64

	# parameters
	l_front::Float64
	l_rear::Float64
	width::Float64

	dt::Float64 

	input_lower_bound::Array{Float64}
	input_upper_bound::Array{Float64}
	state_s_lower_bound::Array{Float64}
	state_s_upper_bound::Array{Float64}

	v_max::Float64
	max_slip_angle::Float64

	mass::Float64
	mu::Float64
	g::Float64
	I_z::Float64
	B::Float64
	C::Float64

	# dynamic or kinematic model
	is_dynamic::Bool

	# states and inputs
	states_xy::Array{Float64}
	states_s::Array{Float64}
	states_true_s::Array{Float64}

	inputs::Array{Float64}
	current_lap::Int64
	current_iteration::Int64
	current_input::Array{Float64}

	final_s::Array{Float64}
	final_xy::Array{Float64}
	final_input::Array{Float64}

	optimal_inputs::Array{Float64}
	predicted_xy::Array{Float64}
	predicted_true_xy::Array{Float64}
	predicted_s::Array{Float64}
	prev_predicted_s::Array{Float64}

	# num_laps::Int64
	iterations_needed::Array{Int64}
	# trajectories
	# act as memory of all old states
	trajectories_s::Array{Float64}
	trajectories_xy::Array{Float64}
	previous_inputs::Array{Float64}
	# selected states
	# act as memory of selected old states
	selected_states_s::Array{Float64}
	selected_states_xy::Array{Float64}
	selected_states_cost::Array{Float64}

	all_selected_states::Dict{Int64,Array{Tuple{Int64,UnitRange{Int64}},1}}
	all_predictions::Array{Float64}

	num_loaded_laps::Int64
	color::AbstractString
	weights_states::Array{Float64}

	updated_state::Bool
	xy_predictions::xy_prediction

	state_initialized::Bool
	blocked::Bool

	theta_vx::Array{Float64}
	theta_vy::Array{Float64}
	theta_psi_dot::Array{Float64}

	t_vx_log::Array{Float64}
	t_vy_log::Array{Float64}
	t_psi_log::Array{Float64}

	t_counter::Int64

	thetas::theta
	selection::selected_states

	dynamic_states::Array{Float64}
	next_dynamics::Array{Float64}
	loaded_sys_id_data::Bool

	time_inputs::Float64
	time_states::Float64

	num_sessions::Int64
	current_session::Int64
	dynamics::Array{Float64}
	counter::Array{Int64}

	acc::Float64

	Agent() = new()
end

function init!(agent::Agent, index::Int64, track::Track, 
			   maximum_num_iterations::Int64, horizon::Int64, num_laps::Int64, 
			   num_considered_states::Int64, num_loaded_laps::Int64, 
			   v_max::Float64, color::AbstractString)
	node_name = get_name()
	agent.index = get_param(node_name * "/index")
	# agent.index = index
	agent.dt = 0.1  # TODO: define somewhere else!? maybe config file?

	agent.l_front = 0.125
	agent.l_rear = 0.125
	agent.width = 0.1

	agent.input_lower_bound = [(- 0.6) (- pi / 3)]
	agent.input_upper_bound = [0.6 (pi / 3)]
	agent.state_s_lower_bound = [(- Inf) (- Inf) (- Inf) (- Inf) 0.0 0.0]
	agent.state_s_upper_bound = [Inf Inf Inf Inf Inf Inf]

	agent.v_max = v_max
	agent.max_slip_angle = 10

	agent.mass = get_param(node_name * "/mass")
	println("mass: ", agent.mass)
	# agent.mass = 1.98
	agent.mu = 0.85
	agent.g = 9.81
	agent.I_z = 0.03
	agent.B = 6.0
	agent.C = 1.6 

	agent.is_dynamic = true

	# TODO: iterations_needed here or in simulator?
	agent.iterations_needed = zeros(num_laps + num_loaded_laps)
	agent.trajectories_s = zeros(num_laps + num_loaded_laps, 
								 maximum_num_iterations, 6)
	agent.trajectories_xy = zeros(num_laps + num_loaded_laps, 
								  maximum_num_iterations, 6)
	agent.previous_inputs = zeros(num_laps + num_loaded_laps, 
								  maximum_num_iterations, 2)

	agent.selected_states_s = zeros(num_considered_states, 6)
	agent.selected_states_xy = zeros(num_considered_states, 6)
	agent.selected_states_cost = zeros(num_considered_states)

	# agent.all_selected_states = Dict([i => (0, 1 : 10) for i = 1 : 6000]) 
	agent.all_selected_states = Dict([i => [(j, 1 : 10) for j = 1 : NUM_CONSIDERED_LAPS] for i = 1 : 6000])
	agent.all_predictions = zeros(6000, horizon + 1, 6)

	agent.num_loaded_laps = num_loaded_laps

	agent.current_session = 1
	agent.num_sessions = 1
	agent.counter = ones(agent.num_sessions)
	agent.dynamics = zeros(agent.num_sessions, 6000, 5)

	if MODE == "learning"
		filename = get_most_recent(node_name[2 : end], "path_following", 
								   INITIALIZATION_TYPE)
		# filename = "/home/lukas/simulations/" * get_name() * 
		#		   "_path_following_" * INITIALIZATION_TYPE * ".jld"
		# filename = "/home/lukas/simulations/lmpc_test.jld"
		println("LOADING: ", filename)
		load_trajectories!(agent, filename)
	end

	# x-y state: z_xy = [x y v_x v_y psi psi_dot]
	agent.states_xy = zeros(maximum_num_iterations, 6)
	# s-e_y state: z_s = [s e_y e_psi psi_dot v_x v_y]
	agent.states_s = zeros(maximum_num_iterations, 6)
	# input: u = [acceleration steering_angle]
	agent.inputs = zeros(maximum_num_iterations, 2)

	# current state and input
	agent.current_iteration = 1
	agent.current_input = zeros(1, 2)
	agent.final_input = zeros(1, 2)
	agent.final_s = zeros(1, 6)
	agent.final_xy = zeros(1, 6)

	agent.optimal_inputs = zeros(horizon, 2)
	agent.predicted_xy = zeros(horizon + 1, 6)
	# agent.predicted_true_xy = zeros(horizon + 1, 6)
	agent.predicted_s = zeros(horizon + 1, 6)
	agent.prev_predicted_s = zeros(horizon + 1, 6)

	# set the velocity for the first state unequal to zero
	# initial_psi = agent.states_xy[1, 5]
	# agent.states_xy[1, 3 : 4] = 0.4 * [cos(initial_psi) sin(initial_psi)]
	# make sure the states for xy and s are initialized equally!
	# agent.states_s[1, :] = xy_to_s(track, agent.states_xy[1, :])
	# agent.states_true_s[1, 5] = 0.4

	agent.color = get_param(node_name * "/color")
	println("color: ", agent.color)
	# agent.color = color
	agent.current_lap = 1
	agent.weights_states = ones(num_considered_states)

	agent.updated_state = false
	agent.xy_predictions = xy_prediction()

	agent.state_initialized = true
	agent.blocked = false

	agent.theta_vx = zeros(Float64, 3)
	agent.theta_vy = zeros(Float64, 4)
	agent.theta_psi_dot = zeros(Float64, 3)

	agent.t_vx_log = zeros(1000, 3)
	agent.t_vy_log = zeros(1000, 4)
	agent.t_psi_log = zeros(1000, 3)

	agent.t_counter = 0

	agent.thetas = theta()
	agent.selection = selected_states()

	agent.dynamic_states = zeros(1, 5)
	agent.next_dynamics =  zeros(1, 3)
	agent.loaded_sys_id_data = false

	agent.time_inputs = 0.0
	agent.time_states = 0.0

	# agent.num_sessions = 1
	# agent.current_session = 1
	# TODO: Determine a good amount of saved dynamics.
	# agent.dynamics = zeros(agent.num_sessions, 2000, 5)  
	# agent.counter = [1]

	agent.acc = 0.0
end

function get_state_xy(agent::Agent, iteration::Int64)
	return agent.states_xy[iteration, :]
end

function get_state_s(agent::Agent, iteration::Int64)
	return agent.states_s[iteration, :]
end

function get_current_s(agent::Agent)
	return agent.states_s[agent.current_iteration, :]
end

function get_current_xy(agent::Agent)
	return agent.states_xy[agent.current_iteration, :]
end

function get_state_true_s(agent::Agent, iteration::Int64)
	return agent.states_true_s[iteration, :]
end

function set_state_xy!(agent::Agent, xy_state::Array{Float64}, track::Track)
	# TODO: only sets the initial state
	agent.states_xy[1, :] = xy_state
	agent.states_s[1, :] = xy_to_s(track, xy_state)
end

function set_current_xy!(agent::Agent, xy_state::Array{Float64}, track::Track)
    iteration = agent.current_iteration
    agent.states_xy[iteration, :] = xy_state
    agent.states_s[iteration, :] = xy_to_s(track, xy_state)
end

function set_state_s!(agent::Agent, s_state::Array{Float64}, track::Track)
	# TODO: only sets the initial state
	s_state = get_s_coord(track, s_state)
	agent.states_xy[1, :] = s_to_xy(track, s_state)
	agent.states_s[1, :] = s_state
end

function set_current_s!(agent::Agent, s_state::Array{Float64}, track::Track)
    s_state = get_s_coord(track, s_state)
    iteration = agent.current_iteration
    agent.states_xy[iteration, :] = s_to_xy(track, s_state)
    agent.states_s[iteration, :] = s_state
end

function save_trajectories(agent::Agent, filename::ASCIIString)
	println("SAVING")
	num_recorded_laps = agent.num_loaded_laps + agent.current_lap - 1
	jldopen(filename, "w") do file
	    JLD.write(file, "trajectories_s", agent.trajectories_s[1 : num_recorded_laps, : , :])
	    JLD.write(file, "trajectories_xy", agent.trajectories_xy[1 : num_recorded_laps, :, :])
	    JLD.write(file, "previous_inputs", agent.previous_inputs[1 : num_recorded_laps, :, :])
	    JLD.write(file, "dynamics", agent.dynamics)
	    JLD.write(file, "all_selected_states", agent.all_selected_states)
	    JLD.write(file, "all_predictions", agent.all_predictions)
	end
end

function load_trajectories!(agent::Agent, filename::ASCIIString)
    Data = load(filename)
    num_loaded_laps = size(Data["trajectories_s"])[1]
    agent.trajectories_s[1 : num_loaded_laps, :, :] = Data["trajectories_s"]
    agent.trajectories_xy[1 : num_loaded_laps, :, :] = Data["trajectories_xy"]
    agent.previous_inputs[1 : num_loaded_laps, :, :] = Data["previous_inputs"]
    previous_session_dynamics = Data["dynamics"]
    agent.num_sessions = size(previous_session_dynamics, 1) + 1
    agent.current_session = agent.num_sessions
    agent.dynamics = zeros(agent.num_sessions, 6000, 5)
    agent.dynamics[1 : agent.num_sessions - 1, :, :] = previous_session_dynamics
    agent.counter = zeros(agent.num_sessions)
    agent.counter[end] = 1
    for i = 1 : agent.num_sessions - 1
    	agent.counter[i] = findmin(sumabs(agent.dynamics[i, :, :], 3))[2]
    end
    determine_needed_iterations!(agent)
 end

 function load_trajectories!(agent::Agent, filenames::Array{ASCIIString})
 	num_previously_loaded = 0
 	for filename in filenames
 		Data = load(filename)
 		num_loaded_laps = size(Data["trajectories_s"])[1] + num_previously_loaded
 		println(num_loaded_laps)
 		agent.trajectories_s[num_previously_loaded + 1: num_loaded_laps, :, :] = Data["trajectories_s"]
    	agent.trajectories_xy[num_previously_loaded + 1: num_loaded_laps, :, :] = Data["trajectories_xy"]
    	num_previously_loaded = num_loaded_laps
 	end

 	determine_needed_iterations!(agent)
 end

function select_states(agent::Agent, chosen_lap::Int64)
	# Always using the previous lap
	# TODO: smarter selection of states
	# TODO: select from latest lap
	#chosen_lap = 2
	# println(agent.trajectories_s)
	agent.iterations_needed[chosen_lap] = findmin(sumabs(agent.trajectories_s[chosen_lap, :, :], (1, 3)))[2] - 1
	# println("SUM: ", sumabs(agent.trajectories_s[2, :, :], (1, 3)))
	needed_iterations = agent.iterations_needed[chosen_lap]
	# println("needed iterations: ", needed_iterations)
	# println("test: ", agent.trajectories_s[chosen_lap, needed_iterations - 2 : needed_iterations + 2, :])
	# println("test: ", agent.trajectories_s[chosen_lap, needed_iterations - 1, :])

	# exit()
	num_considered_states = size(agent.selected_states_s)[1]
	# println("num considered states: ", num_considered_states)

	horizon = size(agent.optimal_inputs)[1]

	iteration = agent.current_iteration
	first_iteration = findmin(colwise(Euclidean(), 
							  agent.trajectories_s[chosen_lap, 1 : needed_iterations, 1], 
							  [agent.states_s[iteration, 1]]))[2]
	if first_iteration == needed_iterations
		first_iteration = 1
	end

	# println("first iteration: ", first_iteration)

	# TODO: check if necessary
	#=
	if needed_iterations <= num_considered_states
		agent.selected_states_s = squeeze(agent.trajectories_s[chosen_lap, 
										  first_iteration : first_iteration + needed_iterations - 1, :], 1)
	else
		agent.selected_states_s = squeeze(agent.trajectories_s[chosen_lap, 
										  first_iteration : first_iteration + num_considered_states - 1, :], 1)
		agent.selected_states_xy = squeeze(agent.trajectories_xy[chosen_lap, 
										   first_iteration : first_iteration + num_considered_states - 1, :], 1)
	end
	=#
	if needed_iterations <= num_considered_states
		agent.selected_states_s = squeeze(agent.trajectories_s[chosen_lap, 
										  first_iteration : first_iteration + 2 * horizon - 1, :], 1)
	else
		agent.selected_states_s = squeeze(agent.trajectories_s[chosen_lap, 
										  first_iteration : first_iteration + 2 * horizon - 1, :], 1)
		agent.selected_states_xy = squeeze(agent.trajectories_xy[chosen_lap, 
										   first_iteration : first_iteration + 2 * horizon - 1, :], 1)
	end

	agent.selected_states_s = repmat(agent.selected_states_s, 5)
	agent.selected_states_xy = repmat(agent.selected_states_xy, 5)
end

function select_states(agent::Agent)
	# println("TEST: ", sumabs(agent.trajectories_s[:, :, :], (2, 3)))
	num_recorded_laps = NUM_LOADED_LAPS + agent.current_lap - 1
	# num_recorded_laps = findmin(sumabs(agent.trajectories_s[:, :, :], (2, 3)))[2] - 1

	possible_trajectory_list = nothing
	closest_index = zeros(Int64, num_recorded_laps)

	horizon = size(agent.optimal_inputs)[1]
	num_considered_states = size(agent.selected_states_s)[1]
	num_buffer = NUM_STATES_BUFFER
	iteration = agent.current_iteration

	selected_laps = zeros(Int64, NUM_CONSIDERED_LAPS)

	needed_iteration_array = zeros(num_recorded_laps)

	# println("num recorded laps: ", num_recorded_laps)

	# find the fastest lap
	min_iterations = [size(agent.states_xy)[1], 0]

	# for i = 1 : num_recorded_laps
	# Do not select from the first overall lap, because there are no states, 
	# which could be appended in the beginning
	for i = 2 : num_recorded_laps
		if i == NUM_LOADED_LAPS
			continue
		end
		needed_iterations = agent.iterations_needed[i]

		#=
		if i > NUM_LOADED_LAPS
			# Because this number of states is previously added, but not 
			# accounted for
			needed_iterations += num_considered_states
		end
		=#

		if agent.trajectories_s[i, num_buffer + 1, 1] <= 2 * agent.dt * agent.v_max
			needed_iteration_array[i] = needed_iterations
		else
			needed_iteration_array[i] = needed_iterations + Int64(ceil(2 * agent.dt * agent.v_max))
		end

		# index_closest_state = findmin(colwise(Euclidean(), 
		# 							  agent.trajectories_s[i, num_considered_states + 1 : needed_iterations + 2 * num_considered_states, 1],
		# 							  [agent.states_s[iteration, 1]]))[2]
		index_closest_state = findmin(abs(agent.trajectories_s[i, num_buffer + 1 : needed_iterations + 2 * num_buffer, 1]
									      - agent.states_s[iteration, 1]))[2] + num_buffer
		
		closest_index[i] = Int64(index_closest_state)
		# determine reachability from current state

		# current_vel = agent.states_s[iteration, 4]
		current_vel = sqrt(agent.states_s[iteration, 5]^2 + agent.states_s[iteration, 6]^2)
		# current_vel = agent.states_s[iteration, 5]

		# println("Current velocity: ", current_vel)
		#max_diff_vel = horizon * agent.dt * agent.input_upper_bound[1] 
		max_diff_vel = (horizon + 4) * agent.dt * agent.input_upper_bound[1] 

		# println("max_diff_vel", max_diff_vel)

		# recorded_vel_ahead = agent.trajectories_s[i, index_closest_state + horizon, 4]
		recorded_vel_ahead = sqrt(agent.trajectories_s[i, index_closest_state + horizon, 5]^2 +
								  agent.trajectories_s[i, index_closest_state + horizon, 6]^2)
		# recorded_vel_ahead = agent.trajectories_s[i, index_closest_state + horizon, 5]

		# println("recorded_vel_ahead", agent.trajectories_s[i, index_closest_state : index_closest_state + horizon + 5, 4])
		# TODO: Figure out a better way to do this!
		# println("recorded vel", recorded_vel_ahead)
		# println("current vel", current_vel)
		# println("max_diff_vel", max_diff_vel)
		
		if current_vel - max_diff_vel <= recorded_vel_ahead && current_vel + 
			max_diff_vel >= recorded_vel_ahead

			travel_distance = horizon * agent.dt * (current_vel ) # + 0.5 * max_diff_vel)

			if travel_distance > abs(agent.trajectories_s[i, index_closest_state, 1] - agent.states_s[iteration, 1])
				if possible_trajectory_list == nothing
					 possible_trajectory_list = [i]
				else 
					append!(possible_trajectory_list, [i])
				end
			end
		end
		
		#=
		if possible_trajectory_list == nothing
			 possible_trajectory_list = [i]
		else 
			append!(possible_trajectory_list, [i])
		end
		=#
	end

	#=
	if possible_trajectory_list == nothing
		for i = 1 : num_recorded_laps
			println(agent.trajectories_s[i, :, 4])
		end
	end
	=#

	# println("Possible trajectories:", possible_trajectory_list)

	if num_recorded_laps <= NUM_CONSIDERED_LAPS  # && possible_trajectory_list == nothing
		possible_trajectory_list = collect(1 : num_recorded_laps)
		possible_trajectory_list[1] = 2
	end

	if possible_trajectory_list == nothing
		possible_trajectory_list = collect(num_recorded_laps: - 1 : 1)
		if agent.current_lap == 1 && NUM_LOADED_LAPS > 0
			possible_trajectory_list[end] = 2
		end
	end
	@assert possible_trajectory_list != nothing

	sorted_iterations = sortperm(needed_iteration_array)
	num_possible_trajectories = length(possible_trajectory_list)

	println("num possible_trajectory: ", num_possible_trajectories)

	i, j = 1, 1
	while j <= NUM_CONSIDERED_LAPS
		if num_possible_trajectories <= NUM_CONSIDERED_LAPS && j >= num_possible_trajectories
			selected_laps[j] = possible_trajectory_list[end]
			j += 1
		elseif sorted_iterations[i] in possible_trajectory_list
			selected_laps[j] = sorted_iterations[i]
			j += 1
		end
		i += 1
	end

	# Make sure, that certain laps are not selected. 
	if 1 in selected_laps && NUM_LOADED_LAPS > 0 
		for i = 1 : size(selected_laps, 1)
			if selected_laps[i] == 1
				selected_laps[i] = 2
			end
		end
	end
	if NUM_LOADED_LAPS > 0 && NUM_LOADED_LAPS in selected_laps
		for i = 1 : size(selected_laps, 1)
			if selected_laps[i] == NUM_LOADED_LAPS
				selected_laps[i] = 2
			end
		end
	end
	# selected_laps = round(Int64, num_recorded_laps) * ones(Int64, 5)
	println("selected_laps: ", selected_laps)

	#=
	if agent.current_lap > 2
		if !(num_recorded_laps in selected_laps)
			lap = num_recorded_laps
			println("current_s: ", agent.states_s[iteration, 1])
			println("closest_index: ", closest_index[lap])
			println("chosen s: ", agent.trajectories_s[lap, closest_index[lap], 1])
			println("prev states: ", agent.trajectories_s[lap, 1 : agent.iterations_needed[lap] + num_considered_states, 1])
			println("Previous lap not in selection anymore")
			exit()
		end
	end
	=#
 
	counter = 1
	iteration_number = 1
	for i in selected_laps
		agent.selected_states_s[counter : counter + 2 * horizon - 1, :] = 
			squeeze(agent.trajectories_s[i, 
					closest_index[i] : closest_index[i] + 2 * horizon - 1, :], 
					1)
		agent.selected_states_xy[counter : counter + 2 * horizon - 1, :] = 
			squeeze(agent.trajectories_xy[i, 
					closest_index[i] : closest_index[i] + 2 * horizon - 1, :], 
					1)		
		# agent.selected_states_cost[counter : counter + 2 * horizon - 1, :] = 
		#	collect(agent.iterations_needed[i] : - 1 : agent.iterations_needed[i] - 2 * horizon + 1)
		agent.selected_states_cost[counter : counter + 2 * horizon - 1, :] = 
			collect(agent.iterations_needed[i] - closest_index[i] + num_buffer : - 1 : 
					agent.iterations_needed[i] - closest_index[i] + num_buffer - 2 * horizon + 1)
		
		agent.all_selected_states[agent.counter[end]][iteration_number] = (i, closest_index[i] : closest_index[i] + 2 * horizon - 1)

		counter += 2 * horizon
		iteration_number += 1		
	end

	# println("COST: ", agent.selected_states_cost)

	println("SELECTED VX MIN: ", findmin(agent.selected_states_s[:, 5])[1])
	println("SELECTED VX MAX: ", findmax(agent.selected_states_s[:, 5])[1])

	identify_system!(agent)
end

function select_states_smartly(agent::Agent, current_adv_e_y::Float64, 
							   track::Track)
	num_recorded_laps = findmin(sumabs(agent.trajectories_s[:, :, :], (2, 3)))[2] - 1
	possible_trajectory_list = nothing
	avg_e_y = nothing
	closest_index = zeros(Int64, num_recorded_laps)

	horizon = size(agent.optimal_inputs)[1]
	num_considered_states = size(agent.selected_states_s)[1]
	iteration = agent.current_iteration

	overtake_on_left = true
	# println("num recorded laps: ", num_recorded_laps)

	# find the fastest lap
	min_iterations = [size(agent.states_xy)[1], 0]

	# TODO: Use adversarial agent's width, 4 could also be changed!!
	current_e_y = agent.states_s[iteration, 2]
	# TODO: using the last e_y in the prediction of the adversarial agent

	current_vel = agent.states_s[iteration, 4]

	if current_adv_e_y > current_e_y 
		if current_adv_e_y + track.width / 2.0 > 4 * agent.width
		    # add states on that side
		    # determine the interval
			interval = [(current_e_y - 2 * agent.width) (current_e_y + 2 * agent.width)]
			overtake_on_left = false
		else 
			interval = [current_adv_e_y (current_adv_e_y + 4 * agent.width)]
			overtake_on_left = true
		end
	else
		if track.width / 2.0 - current_adv_e_y > 4 * agent.width
		# add states on the other side
		# determine the interval
			interval = [(current_e_y - 2 * agent.width) (current_e_y + 2 * agent.width)]
			overtake_on_left = true
		else
			interval = [(current_adv_e_y - 4 * agent.width) current_adv_e_y]
			overtake_on_left = false
		end
	end

	#=
	for i = 1 : 2
		iterated_s = agent.states_s[iteration, 1]
		state_xy = zeros(2 * horizon, 6)
		for j = 1 : 2 * horizon
			state_s = [iterated_s interval[i] 0.0 0.0]
			state_xy[j, :] = s_to_xy(track, state_s)
			iterated_s += current_vel * agent.dt
		end
		plot(state_xy[:, 1], state_xy[:, 2], "go")
	end
	=#

	for i = 1 : num_recorded_laps
		if i <= agent.num_loaded_laps
			agent.iterations_needed[i] = findmin(sumabs(agent.trajectories_s[i, :, :], (1, 3)))[2] - 1 - num_considered_states
		end
		needed_iterations = agent.iterations_needed[i]

		if needed_iterations <= min_iterations[1]
			if agent.trajectories_s[1, 1] <= 2 * agent.dt * agent.v_max
				min_iterations = [needed_iterations, i]
			end
		end

		# println(needed_iterations)
		index_closest_state = findmin(colwise(Euclidean(), 
									  agent.trajectories_s[i, 1 : needed_iterations, 1],
									  [agent.states_s[iteration, 1]]))[2]
		closest_index[i] = Int64(index_closest_state)
		# determine reachability from current state

		max_diff_vel = horizon * agent.dt * agent.input_upper_bound[1]

		recorded_vel_ahead = agent.trajectories_s[i, index_closest_state + horizon, 4]
		if current_vel - max_diff_vel <= recorded_vel_ahead && current_vel + 
			max_diff_vel >= recorded_vel_ahead

			travel_distance = horizon * agent.dt * current_vel

			if travel_distance > abs(agent.trajectories_s[i, index_closest_state, 1] - agent.states_s[iteration, 1])

				# mutual information 
				# println(vec(agent.trajectories_s[i, index_closest_state : index_closest_state + 2 * horizon, 2]))
				# println(vec(agent.trajectories_s[num_recorded_laps, index_closest_state : index_closest_state + 2 * horizon, 2]))
				# mi_e_y = get_mutual_information(vec(agent.trajectories_s[num_recorded_laps, index_closest_state : index_closest_state + 2 * horizon, 2]),
				# 							  	vec(agent.trajectories_s[i, index_closest_state : index_closest_state + 2 * horizon, 2]))
				# println(mi_e_y)

				# println(agent.trajectories_s[i, index_closest_state : index_closest_state + 2 * horizon - 1, 2])
				visited_e_ys = agent.trajectories_s[i, index_closest_state : index_closest_state + 2 * horizon - 1, 2]
				mean_e_y = mean(visited_e_ys)
				# sorted_e_ys = sort(vec(visited_e_ys))

				# if mean_e_y >= interval[1] && mean_e_y <= interval[2]
					if overtake_on_left
						if any(visited_e_ys .< interval[1])
							continue
						end
					else
						if any(visited_e_ys[1] .> interval[1])
							continue
						end
					end

					# plot(agent.trajectories_xy[i, index_closest_state : index_closest_state + 2 * horizon, 1], 
					#	 agent.trajectories_xy[i, index_closest_state : index_closest_state + 2 * horizon, 2], "go")
					if possible_trajectory_list == nothing
						 possible_trajectory_list = [i]
						 avg_e_y = [mean_e_y]
					else 
						append!(possible_trajectory_list, [i])
						append!(avg_e_y, [mean_e_y])
					end
				# end
			end
		end
	end

	#=
	while length(mi_vector) <= 4
		append!(mi_vector, [mi_vector[1]])
		append!(possible_trajectory_list, [possible_trajectory_list[1]])
	end
	=#

	if avg_e_y == nothing
		select_states(agent)
	else
		num_possible_trajectories = length(possible_trajectory_list)

		sorted_e_y = sortperm(avg_e_y)
		# println(avg_e_y[sorted_e_y])
		min_max_e_y = [possible_trajectory_list[sorted_e_y[1]]; 
					   possible_trajectory_list[sorted_e_y[end]]]

		if num_possible_trajectories >= NUM_CONSIDERED_LAPS - 1
			if overtake_on_left
				# println("OVERTAKE ON THE LEFT")
				possible_trajectory_list = possible_trajectory_list[sorted_e_y[1 : NUM_CONSIDERED_LAPS - 1]]
				# possible_trajectory_list = possible_trajectory_list[sorted_e_y[end : - 1 : end - (NUM_CONSIDERED_LAPS - 2)]]
			else 
				possible_trajectory_list = possible_trajectory_list[sorted_e_y[end : - 1 : end - (NUM_CONSIDERED_LAPS - 2)]]
				# possible_trajectory_list = possible_trajectory_list[sorted_e_y[1 : NUM_CONSIDERED_LAPS - 1]]
			end
		# end

		# if num_possible_trajectories > 2
		# 	possible_trajectory_list = possible_trajectory_list[sorted_e_y[randperm(num_possible_trajectories - 2) + 1]]
		else
			possible_trajectory_list = possible_trajectory_list[randperm(num_possible_trajectories)]
		end

		while length(possible_trajectory_list) < 4
			append!(possible_trajectory_list, [possible_trajectory_list[randperm(length(possible_trajectory_list))[1]]])
		end

		# println("interval: [$(interval[1]), $(interval[2])")

		# println(avg_e_y[sorted_e_y])
		# possible_trajectory_list = possible_trajectory_list[1 : 2]
		# append!(possible_trajectory_list, min_max_e_y)
		append!(possible_trajectory_list, [min_iterations[2]])

		# println("selected laps: ", possible_trajectory_list)

		counter = 1
		for i in possible_trajectory_list
			agent.selected_states_s[counter : counter + 2 * horizon - 1, :] = 
				squeeze(agent.trajectories_s[i, 
						closest_index[i] : closest_index[i] + 2 * horizon - 1, :], 
						1)
			agent.selected_states_xy[counter : counter + 2 * horizon - 1, :] = 
				squeeze(agent.trajectories_xy[i, 
						closest_index[i] : closest_index[i] + 2 * horizon - 1, :], 
						1)		
			agent.selected_states_cost[counter : counter + 2 * horizon - 1, :] = 
				collect(agent.iterations_needed[i] : - 1 : agent.iterations_needed[i] - 2 * horizon + 1)
				# collect(2 * horizon : - 1 : 1)
			counter += 2 * horizon
		end

		#=
		# adjust weights for fastest trajectory
		for i = 2 * horizon - 1 : - 1 : 0
			if overtake_on_left
				if agent.selected_states_s[end - i, 2] < interval[1]
					agent.weights_states[end - i] = 10
				end
			else
				if agent.selected_states_s[end - i, 2] > interval[2]
					agent.weights_states[end - i] = 10
				end
			end
		end
		=#
	end
end

function states_cost(agent::Agent, chosen_lap::Int64)
	# TODO
	needed_iterations = agent.iterations_needed[chosen_lap]
	iteration = agent.current_iteration
	num_considered_states = size(agent.selected_states_s)[1]

	horizon = size(agent.optimal_inputs)[1]

	agent.selected_states_cost = collect(2 * horizon : - 1 : 1)
	agent.selected_states_cost = repmat(agent.selected_states_cost, 5)
	# println(size(agent.selected_states_cost))
end

function identify_system!(agent::Agent)
	num_states = 20  # number of states used for system identification
	num_previous = SYS_ID_BEFORE
	num_after = SYS_ID_AFTER

	# Create the current dynamic state: [v_x v_y psi_dot acc steering]
	current_iteration = agent.current_iteration

	# Mapping: [s, e_y, e_psi, psi_dot, v_x, v_y] --> [v_x v_y psi_dot]
	mapping = [5, 6, 4]
	# println(agent.states_s[current_iteration])
	# println(agent.current_input)
	current_s = agent.states_s[current_iteration, 1]
	v_x, v_y, psi_dot  = agent.states_s[current_iteration, mapping]
	acc, steering = agent.current_input

	current_dynamics = [v_x, v_y, psi_dot, acc, steering]
	# current_dynamics = squeeze(agent.dynamics[agent.num_sessions, max(agent.counter[agent.current_session] - 1, 1), :], (1, 2))
	println("Current dynamics: ", current_dynamics)

	num_recorded_laps = agent.num_loaded_laps + agent.current_lap - 1
	# num_considered_states = size(agent.selected_states_s)[1]
	num_buffer = NUM_STATES_BUFFER

	# Count the number of recorded states
	total_recorded_states = current_iteration - 1

	# Create the dynamics of all the recorded laps
	for i = 1 : num_recorded_laps
		total_recorded_states += agent.iterations_needed[i]
	end

	summed_iterations = cumsum([1; agent.iterations_needed])

	println("Summed iterations: ", summed_iterations)

	#=
	# Loading Shuqi's data for debugging
	if !agent.loaded_sys_id_data
		data = load("/home/lukas/Downloads/efficient_feature_data_process.jld")
		z = data["z_feature"]
		u = data["u_feature"]

		# Initialize dynamic_states
		dynamic_states = zeros(size(z, 1), 5)

		# Mapping: [s, e_y, e_psi, v_x, v_y, psi_dot] --> [v_x v_y psi_dot]
		mapping = [4, 5, 6]
		dynamic_states[:, 1 : 3] = z[:, mapping, 1]
		dynamic_states[:, 4 : 5] = u

		next_dynamics = z[:, mapping, 2]

		agent.loaded_sys_id_data = true
		agent.dynamic_states = dynamic_states
		agent.next_dynamics = next_dynamics
	end

	dynamic_states = agent.dynamic_states
	next_dynamics = agent.next_dynamics

	# Calcualte euclidean distances and sort
	norm_on_dynamics = colwise(Euclidean(), dynamic_states', current_dynamics)
	sorted_indeces = sortperm(norm_on_dynamics)[1 : num_states]

	selected_dynamics = dynamic_states[sorted_indeces, :]
	next_dynamics = next_dynamics[sorted_indeces, :]
	
	# End using Loading Shuqi's data
	=#

	vx_tolerance = 0.1
	acc_tolerance = 0.1
	steering_tolerance = 0.05

	list = []

	for session = 1 : agent.num_sessions
		for i = 1 : agent.counter[session] - 3
			recorded_dynamics = agent.dynamics[session, i, :]
			if recorded_dynamics[1] < v_x + vx_tolerance &&
				recorded_dynamics[1] > v_x - vx_tolerance
				if recorded_dynamics[4] < acc + acc_tolerance &&
					recorded_dynamics[4] > acc - acc_tolerance
					if recorded_dynamics[5] < steering + steering_tolerance &&
						recorded_dynamics[5] > steering - steering_tolerance
						append!(list, [(session, i)])
					end
				end
			end
		end
	end

	length_list = size(list, 1)
	println("LENGTH OF LIST: ", length_list)

	if false
	# if length_list >= SYS_ID_AFTER
	# if MODE == "racing" # || current_dynamics[1] > 1.1
	# if true
		####################################################################
		# Selection based on norm_on_dynamics #
		####################################################################
		println("LIST IS LONG ENOUGH!")
		#=
		# Initialize matrix for all the previous laps
		dynamic_states = zeros(total_recorded_states, 5)

		println("total recorded states: ", total_recorded_states)

		# TODO: Only create once and then add to it. 
		for i = 1 : num_recorded_laps
			previous_dynamics = zeros(agent.iterations_needed[i], 5)
			indeces = num_buffer + 1 : agent.iterations_needed[i] + num_buffer
			previous_dynamics[:, 1 : 3] = agent.trajectories_s[i, indeces, mapping]
			previous_dynamics[:, 4 : 5] = agent.previous_inputs[i, indeces, :]
			dynamic_states[summed_iterations[i] : summed_iterations[i + 1] - 1, :] = previous_dynamics
		end

		if current_iteration > 2
			current_lap_dynamics = zeros(current_iteration - 1, 5)
			indeces = 1 : current_iteration - 1
			current_lap_dynamics[:, 1 : 3] = agent.states_s[indeces, mapping]
			current_lap_dynamics[:, 4 : 5] = agent.inputs[indeces, :]
			dynamic_states[summed_iterations[end] : end, :] = current_lap_dynamics
		end
		=#

		dynamic_states = zeros(length_list, 5)
		next_dynamics = zeros(length_list, 5)
		index = 1

		for (i, j) in list
			dynamic_states[index, :] = agent.dynamics[i, j, :]
			next_dynamics[index, :] = agent.dynamics[i, j + 1, :]
			index += 1
		end

		# println("size dynamic states: ", size(dynamic_states))

		# Calcualte euclidean distances and sort
		norm_on_dynamics = colwise(Euclidean(), dynamic_states[:, 3]', [current_dynamics[3]])
		# norm_on_dynamics = colwise(Euclidean(), dynamic_states[:, 1]', [current_dynamics[1]])
		# norm_on_dynamics = colwise(Euclidean(), dynamic_states', current_dynamics)
		# norm_on_dynamics = colwise(Euclidean(), dynamic_states[:, 1 : 3]', current_dynamics[1 : 3])
		# println("size norm on dynamics: ", size(norm_on_dynamics))
		sorted_indeces = sortperm(norm_on_dynamics)[1 : num_states]
		# println("sorted indeces: ", sorted_indeces)

		#=
		# --> otherwise can't form the difference, just take the previous state then
		for i = 1 : num_states
			if sorted_indeces[i] in (summed_iterations[2 : end] - 1)
				sorted_indeces[i] -= 1
			elseif sorted_indeces[i] == total_recorded_states
				sorted_indeces[i] -= 1
			end
		end
		=#

		#=
		# Delete states with negative velocites or very small velocities. 
		# Shouldn't be relevant
		for i = size(sorted_indeces, 1) : - 1 : 1
			if dynamic_states[sorted_indeces[i], 2] < 1e-5 || 
				dynamic_states[sorted_indeces[i], 3] < 1e-5
				deleteat!(sorted_indeces, i)
			end
		end
		=#

		for i = size(sorted_indeces, 1) : - 1 : 1
			if dynamic_states[sorted_indeces[i], 1] < 1e-5
				deleteat!(sorted_indeces, i)
			end
		end

		println("sorted indeces: ", sorted_indeces)

		# selected_dynamics = dynamic_states[sorted_indeces, :]
		# next_dynamics = dynamic_states[sorted_indeces + 1, :]
		selected_dynamics = dynamic_states[sorted_indeces, :]
		# println("SELECTED dynamics: ", selected_dynamics)
		next_dynamics = next_dynamics[sorted_indeces, :]
		# println("NEXT dynamics: ", next_dynamics)

		#######################################################################
	else
	# elseif MODE == "learning"
		#######################################################################
		# Selection based on previous lap # 
		#######################################################################
		# Find closest states in terms of s

		# previous_lap = agent.current_lap - 1 + NUM_LOADED_LAPS
		# Use previous lap for now, but only if it is not in:
		# {1, NUM_LOADED_LAPS, NUM_LOADED_LAPS + 1}
		@assert NUM_LOADED_LAPS > 1
		previous_lap = agent.current_lap - 1 + NUM_LOADED_LAPS
		if previous_lap in [NUM_LOADED_LAPS, NUM_LOADED_LAPS + 1]
			if previous_lap == NUM_LOADED_LAPS
				previous_lap -= 1
			elseif previous_lap == NUM_LOADED_LAPS + 1
				previous_lap -= 2
			end
		end

		println("previous lap: ", previous_lap)

		previous_indeces = num_buffer + 1 : agent.iterations_needed[previous_lap] + num_buffer
		previous_s = agent.trajectories_s[previous_lap, previous_indeces, 1]

		distance_to_s = colwise(Euclidean(), previous_s, [current_s])
		closest_index = findmin(distance_to_s)[2]
		selected_indeces = collect((closest_index - num_previous : closest_index + num_after) + num_buffer)

		dynamic_states = squeeze(agent.trajectories_s[previous_lap, :, mapping], 1)
		dynamic_states = [dynamic_states squeeze(agent.previous_inputs[previous_lap, :, :], 1)]

		# Or select all data from the previous lap
		# selected_indeces = previous_indeces[1 : end - 1]

		#=
		# Delete states with negative velocites. Shouldn't be relevant
		for i = size(selected_indeces, 1) : - 1 : 1
			if dynamic_states[selected_indeces[i], 2] < 1e-5 || 
				dynamic_states[selected_indeces[i], 3] < 1e-5
				deleteat!(selected_indeces, i)
			end
		end
		=#

		#=
		tolerance = 0.1
		# Delete states where dt != 0.1
		for i in selected_indeces
			previous_s = agent.trajectories_s[previous_lap, i, 1]
			next_s = agent.trajectories_s[previous_lap, i + 1, 1]
			previous_vx = agent.trajectories_s[previous_lap, i, 5]
			if previous_s + previous_vx * agent.dt + tolerance < next_s
				println(i)
				println("previous s: ", previous_s)
				println("next s: ", next_s)
				deleteat!(selected_indeces, i)
			end
		end
		=#

		selected_dynamics = dynamic_states[selected_indeces, :]
		next_dynamics = dynamic_states[selected_indeces + 1, :]

		current_lap = agent.current_lap + NUM_LOADED_LAPS
		previous_iteration = agent.current_iteration - 1
		if current_lap in [NUM_LOADED_LAPS, NUM_LOADED_LAPS + 1]
			if current_lap == NUM_LOADED_LAPS
				current_lap -= 1
			elseif current_lap == NUM_LOADED_LAPS + 1
				current_lap -= 2
			end
		end

		println("current lap: ", current_lap)

		if previous_iteration >= 2
			indeces = collect((max(1, previous_iteration - SYS_ID_AFTER) : previous_iteration - 1) + num_buffer)
			# indeces = collect((previous_iteration - SYS_ID_AFTER : previous_iteration) + num_buffer)
			#=
			tolerance = 0.1
			for i in indeces
				previous_s = agent.trajectories_s[current_lap, i, 1]
				next_s = agent.trajectories_s[current_lap, i + 1, 1]
				previous_vx = agent.trajectories_s[current_lap, i, 5]
				if previous_s + previous_vx * agent.dt + tolerance < next_s
					println(i)
					println("previous s: ", previous_s)
					println("next s: ", next_s)
					deleteat!(indeces, i - num_buffer)
				end
			end
			=#
			
			previous_dynamic_states = squeeze(agent.trajectories_s[current_lap, :, mapping], 1)
			previous_dynamic_states = [previous_dynamic_states squeeze(agent.previous_inputs[current_lap, :, :], 1)]
			selected_dynamics = [selected_dynamics; previous_dynamic_states[indeces, :]]
			next_dynamics = [next_dynamics; previous_dynamic_states[indeces + 1, :]]

			# println("SELECTED DYNAMICS: ", selected_dynamics)
			# println("NEXT DYNAMICS: ", next_dynamics)
		end		
		#######################################################################
	end

	history_vx = selected_dynamics[:, 1]
	history_vy = selected_dynamics[:, 2]
	history_psi_dot = selected_dynamics[:, 3]
	history_acc = selected_dynamics[:, 4]
	history_steering = selected_dynamics[:, 5]

	# Create vectors for linear regression
	delta_vx = next_dynamics[:, 1] - history_vx
	delta_vy = next_dynamics[:, 2] - history_vy 
	delta_psi_dot = next_dynamics[:, 3] - history_psi_dot

	# println("Delta_Vx: ", delta_vx)

	vy_over_vx = history_vy ./ history_vx
	psi_dot_over_vx = history_psi_dot ./ history_vx
	vy_times_psi_dot = history_vy .* history_psi_dot
	vx_times_psi_dot = history_vx .* history_psi_dot

	# println("vy_times_psi_dot: ", vy_times_psi_dot)

	# Create matrices for linear regression
	X_vx = [vy_times_psi_dot history_vx history_acc]
	# X_vx = [history_vx history_acc]
	X_vy = [vy_over_vx vx_times_psi_dot psi_dot_over_vx history_steering]
	X_psi_dot = [psi_dot_over_vx vy_over_vx history_steering]

	# linear regression

	dummy_vx = zeros(agent.theta_vx)
	dummy_vy = zeros(agent.theta_vy)
	dummy_psi_dot = zeros(agent.theta_psi_dot)

	try
		# agent.theta_vx = inv(X_vx' * X_vx + 1.0 * eye(X_vx' * X_vx)) * X_vx' * delta_vx
		dummy_vx = inv(X_vx' * X_vx) * X_vx' * delta_vx
		# agent.theta_vx = [1.0 agent.theta_vx]
	catch 
		println("history vx: ", history_vx)
		println("Test: ", X_vx)
		println("Singular Xvx: ", X_vx' * X_vx)
		exit()
	end

	try
		# agent.theta_vy = inv(X_vy' * X_vy + 1.0 * eye(X_vy' * X_vy)) * X_vy' * delta_vy
		dummy_vy = inv(X_vy' * X_vy) * X_vy' * delta_vy
	catch
		println("Singular Xvy: ", X_vy' * X_vy)
		exit()
	end

	try 
		# agent.theta_psi_dot = inv(X_psi_dot' * X_psi_dot + 1.0 * eye(X_psi_dot' * X_psi_dot)) * X_psi_dot' * delta_psi_dot
		dummy_psi_dot = inv(X_psi_dot' * X_psi_dot) * X_psi_dot' * delta_psi_dot
	catch
		println("Singular X_psi_dot: ", X_psi_dot' * X_psi_dot)
		exit()
	end

	# agent.theta_vx = [0.05; 0.0; 0.1]
	# agent.theta_vy = [0.0; 0.0; 0.0; 1.0]
	# agent.theta_psi_dot = [0.0; 0.0; 1.0]

	if any(isnan(dummy_vy)) || any(isnan(dummy_psi_dot))
		println("Not updating thetas.")
		println("selected_dynamics: ", selected_dynamics)
		exit()
	else
		agent.theta_vx = dummy_vx
		agent.theta_vy = dummy_vy
		agent.theta_psi_dot = dummy_psi_dot
	end

	println("THETA VX: ", agent.theta_vx)
	println("THETA VY: ", agent.theta_vy)
	println("THETA PSI DOT: ", agent.theta_psi_dot)

	try
		if agent.current_lap > 1
			agent.t_counter += 1
			agent.t_vx_log[agent.t_counter, :] = agent.theta_vx
			agent.t_vy_log[agent.t_counter, :] = agent.theta_vy
			agent.t_psi_log[agent.t_counter, :] = agent.theta_psi_dot
		end	
	end
end

function determine_needed_iterations!(agent::Agent)
	num_buffer = NUM_STATES_BUFFER

	for i = 1 : agent.num_loaded_laps
		agent.iterations_needed[i] = findmin(sumabs(agent.trajectories_s[i, :, :], (1, 3)))[2] - 1 - 2 * num_buffer
		# No data has been appended to the last lap yet.
		if i == NUM_LOADED_LAPS
			agent.iterations_needed[i] += num_buffer
		end
		# No data has been appended to the last lap yet.
		if i == 1 
			agent.iterations_needed[i] += num_buffer
		end
	end
end