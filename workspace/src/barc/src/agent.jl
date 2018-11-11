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
	selected_laps::Array{Int64}
	closest_indeces::Array{Int64}

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
	t_psidot_log::Array{Float64}

	time_log::Array{Float64}

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

	delay_a::Int64
	delay_df::Int64

	not_for_selection::Array{Int64}
    not_for_dynamics::Array{Int64}

    leading::Bool

	Agent() = new()
end

function init!(agent::Agent, index::Int64, track::Track, 
			   maximum_num_iterations::Int64, horizon::Int64, num_laps::Int64, 
			   num_considered_states::Int64, num_loaded_laps::Int64, 
			   v_max::Float64, color::AbstractString)
	node_name = RobotOS.get_name()
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
	agent.selected_laps = zeros(NUM_CONSIDERED_LAPS)
	agent.closest_indeces = zeros(NUM_CONSIDERED_LAPS)

	# agent.all_selected_states = Dict([i => (0, 1 : 10) for i = 1 : 6000]) 
	agent.all_selected_states = Dict([i => [(j, 1 : 10) for j = 1 : NUM_CONSIDERED_LAPS] for i = 1 : 6000])
	agent.all_predictions = zeros(6000, horizon + 1, 6)

	agent.num_loaded_laps = 0

	agent.current_session = 1
	agent.num_sessions = 1
	agent.counter = ones(agent.num_sessions)
	agent.dynamics = zeros(agent.num_sessions, 6000, 5)

	if MODE == "learning"
		filename = ascii(get_most_recent(node_name[2 : end], "path_following", 
								   INITIALIZATION_TYPE))
		println("LOADING: ", filename)
		load_trajectories!(agent, filename)
	elseif MODE == "racing"
		#=
		filename = ascii(get_most_recent(node_name[2 : end], "path_following", 
								   INITIALIZATION_TYPE))
		println("LOADING: ", filename)
		load_trajectories!(agent, filename)
		=#

		filename = ascii(get_most_recent(node_name[2 : end], "path_following", 
								   INITIALIZATION_TYPE))
		filename_center = ascii(get_most_recent(node_name[2 : end], "lmpc", 
								   		  "center"))
		filename_inner = ascii(get_most_recent(node_name[2 : end], "lmpc", 
								   		 "inner"))
		filename_outer = ascii(get_most_recent(node_name[2 : end], "lmpc", 
								   		 "outer"))
		println("LOADING: ", filename)
		println("LOADING: ", filename_center)
		println("LOADING: ", filename_inner)
		println("LOADING: ", filename_outer)

        # files = [filename; filename_center]
		files = [filename; filename_center; filename_inner; filename_outer]
		load_trajectories!(agent, files)
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

	agent.t_vx_log = zeros(6000, 3)
	agent.t_vy_log = zeros(6000, 4)
	agent.t_psidot_log = zeros(6000, 3)

	agent.time_log = zeros(6000)

	agent.t_counter = 0

	agent.thetas = theta()
	agent.selection = selected_states()

	agent.dynamic_states = zeros(1, 5)
	agent.next_dynamics =  zeros(1, 3)
	agent.loaded_sys_id_data = false

	agent.time_inputs = 0.0
	agent.time_states = 0.0

	agent.delay_a = get_param(node_name * "/delay_a")
	agent.delay_df = get_param(node_name * "/delay_df")

	# agent.num_sessions = 1
	# agent.current_session = 1
	# TODO: Determine a good amount of saved dynamics.
	# agent.dynamics = zeros(agent.num_sessions, 2000, 5)  
	# agent.counter = [1]

	agent.acc = 0.0
	agent.leading = false
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
	    JLD.write(file, "theta_vx", agent.t_vx_log)
	    JLD.write(file, "theta_vy", agent.t_vy_log)
	    JLD.write(file, "theta_psi_dot", agent.t_psidot_log)
	    JLD.write(file, "time", agent.time_log)
	end
end

function load_trajectories!(agent::Agent, filename::ASCIIString)
	load_trajectories!(agent, [filename])
end

function load_trajectories!(agent::Agent, filenames::Array{ASCIIString})
	agent.num_sessions = length(filenames) + 1
	num_previously_loaded = 0
	agent.current_session = agent.num_sessions
	agent.dynamics = zeros(agent.num_sessions, 6000, 5)
	agent.counter = zeros(agent.num_sessions)
    agent.counter[end] = 1

    agent.not_for_selection = zeros(agent.num_sessions - 1)
    agent.not_for_dynamics = zeros((agent.num_sessions - 1) * 2 + 1)

    num_laps = 0
    for filename in filenames
	    Data = load(filename)
	    cutoff = 0
	    if contains(filename, "lmpc")
	    	cutoff += 5  # Number of path following laps
	    end
	    num_laps += size(Data["trajectories_s"])[1] - cutoff
	end

	num_laps += NUM_LAPS

	agent.trajectories_s = zeros(num_laps, MAXIMUM_NUM_ITERATIONS, 6)
	agent.trajectories_xy = zeros(num_laps, MAXIMUM_NUM_ITERATIONS, 6)
    agent.previous_inputs = zeros(num_laps, MAXIMUM_NUM_ITERATIONS, 2)
    agent.iterations_needed = zeros(num_laps)

	index = 1
	for filename in filenames
        println(filename)
	    Data = load(filename)
	    cutoff = 1
	    if contains(filename, "lmpc")
	    	cutoff += 5  # Number of path following laps
	    end
	    num_loaded_laps = size(Data["trajectories_s"])[1] + num_previously_loaded - cutoff + 1
	    agent.trajectories_s[num_previously_loaded + 1 : num_loaded_laps, :, :] = Data["trajectories_s"][cutoff : end, :, :]
	    agent.trajectories_xy[num_previously_loaded + 1 : num_loaded_laps, :, :] = Data["trajectories_xy"][cutoff : end, :, :]
	    agent.previous_inputs[num_previously_loaded + 1 : num_loaded_laps, :, :] = Data["previous_inputs"][cutoff : end, :, :]
	    previous_session_dynamics = Data["dynamics"]
	    
	    agent.dynamics[index, :, :] = previous_session_dynamics[end, :, :]
    	agent.counter[index] = findmin(sumabs(agent.dynamics[index, :, :], 3))[2]

    	agent.not_for_selection[index] = num_loaded_laps
    	agent.not_for_dynamics[(index - 1) * 2 + 1] = num_previously_loaded + 1
		agent.not_for_dynamics[(index - 1) * 2 + 2] = num_loaded_laps

	    determine_needed_iterations!(agent, num_previously_loaded + 1 : num_loaded_laps)
	    num_previously_loaded = num_loaded_laps
	    index += 1
	end

	agent.not_for_dynamics[end] = num_previously_loaded + 1

	println(agent.iterations_needed)
	println(agent.not_for_selection)
	println(agent.not_for_dynamics)
	# NUM_LOADED_LAPS = num_previously_loaded
	agent.num_loaded_laps = num_previously_loaded
 end



#=
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
=#

function select_states(agent::Agent, track::Track)
	num_recorded_laps = NUM_LOADED_LAPS + agent.current_lap - 1

	iteration = agent.current_iteration
	prev_s = agent.predicted_s[1, 1]
	current_s = agent.states_s[iteration, 1]

	# Check if we're already in the next lap
	if abs(prev_s - current_s) >= 0.5 * track.total_length && iteration > 1
		num_recorded_laps += 1
	end

	possible_trajectory_list = nothing
	closest_index = zeros(Int64, num_recorded_laps)

	horizon = size(agent.optimal_inputs)[1]
	num_considered_states = size(agent.selected_states_s)[1]
	num_buffer = NUM_STATES_BUFFER

	selected_laps = zeros(Int64, NUM_CONSIDERED_LAPS)

	needed_iteration_array = zeros(num_recorded_laps)

	# find the fastest lap
	min_iterations = [size(agent.states_xy)[1], 0]

	# Do not select from the first overall lap, because there are no states, 
	# which could be appended in the beginning
	for i = 1 : num_recorded_laps
		if i in agent.not_for_selection	
			continue
		end

		if agent.iterations_needed[i] == 0 && i == num_recorded_laps
			needed_iterations = iteration
		else
			needed_iterations = agent.iterations_needed[i]
		end

		if agent.trajectories_s[i, num_buffer + 1, 1] <= 2 * agent.dt * agent.v_max
			needed_iteration_array[i] = needed_iterations
		else
			needed_iteration_array[i] = needed_iterations + Int64(ceil(2 * agent.dt * agent.v_max))
		end

        # println("needed_iterations: ", needed_iterations)
        if needed_iterations == 0
            continue
        end

		index_closest_state = findmin(abs(agent.trajectories_s[i, num_buffer + 1 : needed_iterations + num_buffer, 1]
		 							      - agent.states_s[iteration, 1]))[2] + num_buffer

		index_closest_state += SELECTION_SHIFT
		closest_index[i] = Int64(index_closest_state)

		# determine reachability from current state
		current_vel = sqrt(agent.states_s[iteration, 5]^2 + agent.states_s[iteration, 6]^2)
		max_diff_vel = horizon * agent.dt * agent.input_upper_bound[1] 

		#=
		if MODE == "racing" && agent.current_lap == 2
			max_diff_vel = 0.3
		end
		=#

		recorded_vel_ahead = sqrt(agent.trajectories_s[i, index_closest_state + horizon, 5]^2 +
								  agent.trajectories_s[i, index_closest_state + horizon, 6]^2)
		
		
        # if MODE == "racing"
        #     v_max = 1.5
        #     if any(agent.trajectories_s[i, index_closest_state + (0 : horizon - 1), 5] .> v_max)
        #         continue
        #     end
        # end

		if current_vel - max_diff_vel <= recorded_vel_ahead && current_vel + 
			max_diff_vel >= recorded_vel_ahead

			travel_distance = horizon * agent.dt * (current_vel )

			if travel_distance > abs(agent.trajectories_s[i, index_closest_state, 1] - agent.states_s[iteration, 1])
				if possible_trajectory_list == nothing
					 possible_trajectory_list = [i]
				else 
					append!(possible_trajectory_list, [i])
				end
			end
		end
	end

	if num_recorded_laps <= NUM_CONSIDERED_LAPS
		possible_trajectory_list = collect(1 : num_recorded_laps)
		possible_trajectory_list[1] = 2
	end

	if possible_trajectory_list == nothing
		possible_trajectory_list = collect(num_recorded_laps : - 1 : 1)
		for i = 1 : length(possible_trajectory_list)
			if possible_trajectory_list[i] in agent.not_for_selection
				possible_trajectory_list[i] = 2
			end
		end
	end
	
	@assert possible_trajectory_list != nothing

	sorted_iterations = sortperm(needed_iteration_array)
	num_possible_trajectories = length(possible_trajectory_list)

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

	if MODE == "learning"
		if !(num_recorded_laps in selected_laps)
			dummy_list = [num_recorded_laps]
			append!(dummy_list, selected_laps[1 : NUM_CONSIDERED_LAPS - 1])
			possible_trajectory_list = dummy_list
		end
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

	println("selected_laps: ", selected_laps)

	min_iteration = round(Int64, findmin(agent.iterations_needed[selected_laps])[1])

	counter = 1
	iteration_number = 1
	for i in selected_laps
		agent.selected_laps[iteration_number] = i
		agent.selected_states_s[counter : counter + round(Int64, NUM_HORIZONS * horizon) - 1, :] = 
			squeeze(agent.trajectories_s[i, 
					closest_index[i] : closest_index[i] + round(Int64, NUM_HORIZONS * horizon) - 1, :], 
					1)
		agent.selected_states_xy[counter : counter + round(Int64, NUM_HORIZONS * horizon) - 1, :] = 
			squeeze(agent.trajectories_xy[i, 
					closest_index[i] : closest_index[i] + round(Int64, NUM_HORIZONS * horizon) - 1, :], 
					1)		

		
		# Lukas' cost
		agent.selected_states_cost[counter : counter + round(Int64, NUM_HORIZONS * horizon) - 1, :] = 
			collect((round(Int64, NUM_HORIZONS * horizon) : - 1 : 1) + agent.iterations_needed[i] - min_iteration) 
		
		
        
		# # 1. new cost
		# cost = collect(0 : - 1 : - (round(Int64, NUM_HORIZONS * horizon) - 1)) + agent.iterations_needed[i] - closest_index[i]
		# if any(agent.predicted_s[:, 1] .> track.total_length)
		# 	if i < NUM_LOADED_LAPS + agent.current_lap - 1
		# 		cost += agent.iterations_needed[i + 1]
		# 	else 
		# 		# find state where finish line is crossed
		# 		finish_line_crossed_idx = findmax(agent.predicted_s .> track.total_length)[2]
		# 		cost += agent.current_iteration + finish_line_crossed_idx - 1
		# 	end
		# end
		# agent.selected_states_cost[counter : counter + round(Int64, NUM_HORIZONS * horizon) - 1, :] = cost

		#=
		# 2. new cost
		cost = collect(0 : - 1 : - (round(Int64, NUM_HORIZONS * horizon) - 1)) + agent.iterations_needed[i] - closest_index[i]
		safe_set = squeeze(agent.trajectories_s[i, 
					closest_index[i] : closest_index[i] + round(Int64, NUM_HORIZONS * horizon) - 1, :], 
					1)
		beyond_finish_line = safe_set[:, 1] .> track.total_length
		cost[beyond_finish_line] = 0
		agent.selected_states_cost[counter : counter + round(Int64, NUM_HORIZONS * horizon) - 1, :] = cost
		=#
		
		# Ugo's cost
		#=
		agent.selected_states_cost[counter : counter + round(Int64, NUM_HORIZONS * horizon) - 1, :] = 
			collect(0 : - 1 : - (round(Int64, NUM_HORIZONS * horizon) - 1)) + agent.iterations_needed[i] - closest_index[i]
		=#

		agent.all_selected_states[agent.counter[end]][iteration_number] = (i, closest_index[i] : closest_index[i] + round(Int64, NUM_HORIZONS * horizon) - 1)

		counter += round(Int64, NUM_HORIZONS * horizon)
		iteration_number += 1		
	end

	agent.closest_indeces = closest_index[agent.selected_laps]

	for i = 1 : num_considered_states
		if abs(agent.selected_states_s[i, 2]) > TRACK_WIDTH / 2
			agent.selected_states_cost[i] += 20
		end
	end

	laps_for_sysid = selected_laps[1 : 2]
	identify_system!(agent, laps_for_sysid)
end


function select_states_smartly(agent::Agent, adv_e_y::Float64, adv_s::Array{Float64}, track::Track)
	num_recorded_laps = NUM_LOADED_LAPS + agent.current_lap - 1
	println("AGENT $(agent.index) CURRENT LAP: ", agent.current_lap)
	println("AGENT $(agent.index) NUM RECORDED LAPS: ", num_recorded_laps)
	println("AGENT $(agent.index) NUM LOADED LAPS: ", NUM_LOADED_LAPS)

	iteration = agent.current_iteration
	prev_s = agent.predicted_s[1, 1]
	current_s = agent.states_s[iteration, 1]

	# Check if we're already in the next lap
	#=
	if abs(prev_s - current_s) >= 0.5 * track.total_length && iteration > 1
		num_recorded_laps += 1
	end
	=#

	horizon = size(agent.optimal_inputs)[1]
	num_considered_states = size(agent.selected_states_s)[1]
	num_buffer = NUM_STATES_BUFFER

	selected_laps = zeros(Int64, NUM_CONSIDERED_LAPS)

	needed_iteration_array = zeros(num_recorded_laps)

	possible_trajectory_list = nothing
	avg_e_y = nothing
	closest_index = zeros(Int64, num_recorded_laps)

	overtake_on_left = true

	# find the fastest lap
	min_iterations = [size(agent.states_xy)[1], 0]

	current_e_y = agent.states_s[iteration, 2]
	current_vel = agent.states_s[iteration, 5]

	adv_s_1 = adv_s[1]
	adv_s_2 = adv_s[2]

	alpha = 4.0

	if adv_e_y > current_e_y 
		if adv_e_y + track.width / 2.0 > alpha * agent.width
		    # add states on that side
		    # determine the interval
		    ey_max = min(adv_e_y, current_e_y + alpha / 2.0 * agent.width)
		    ey_min = - track.width / 2.0

		    interval = [ey_min ey_max]
			overtake_on_left = true
		else 
			ey_max = track.width / 2.0
		    ey_min = adv_e_y + alpha / 2.0 * agent.width  # max(adv_e_y - alpha * agent.width, - track.width)
		    interval = [ey_min ey_max]
			overtake_on_left = false
		end
	else
		if track.width / 2.0 - adv_e_y > alpha * agent.width
		# add states on the other side
		# determine the interval
			ey_max = track.width / 2.0
		    ey_min = max(adv_e_y, current_e_y - alpha / 2.0 * agent.width)
		    interval = [ey_min ey_max]
			overtake_on_left = false
		else
			ey_max = adv_e_y - alpha / 2.0 * agent.width
		    ey_min = - track.width / 2.0
			interval = [ey_min ey_max]
			overtake_on_left = true
		end
	end

	for i = 1 : num_recorded_laps
		if i in agent.not_for_selection	
			continue
		end

		if agent.iterations_needed[i] == 0 && i == num_recorded_laps
			needed_iterations = iteration 
		else
			needed_iterations = agent.iterations_needed[i]
		end

		if agent.trajectories_s[i, num_buffer + 1, 1] <= 2 * agent.dt * agent.v_max
			needed_iteration_array[i] = needed_iterations
		else
			needed_iteration_array[i] = needed_iterations + Int64(ceil(2 * agent.dt * agent.v_max))
		end

		index_closest_state = findmin(abs(agent.trajectories_s[i, num_buffer + 1 : needed_iterations + num_buffer, 1]
		 							      - agent.states_s[iteration, 1]))[2] + num_buffer

		index_closest_state += SELECTION_SHIFT
		closest_index[i] = Int64(index_closest_state)

		# determine reachability from current state
		current_vel = sqrt(agent.states_s[iteration, 5]^2 + agent.states_s[iteration, 6]^2)
		max_diff_vel = horizon * agent.dt * agent.input_upper_bound[1] 
		recorded_vel_ahead = sqrt(agent.trajectories_s[i, index_closest_state + horizon, 5]^2 +
								  agent.trajectories_s[i, index_closest_state + horizon, 6]^2)

        # v_max = 1.5
        # if any(agent.trajectories_s[i, index_closest_state + (0 : horizon - 1), 5] .> v_max)
        #     continue
        # end

		if current_vel - max_diff_vel <= recorded_vel_ahead && current_vel + 
			max_diff_vel >= recorded_vel_ahead

			travel_distance = horizon * agent.dt * current_vel

			if travel_distance > abs(agent.trajectories_s[i, index_closest_state, 1] - agent.states_s[iteration, 1])
				visited_s = agent.trajectories_s[i, index_closest_state : index_closest_state + 2 * horizon - 1, 1]
				adv_index_1 = findmin(abs(visited_s - adv_s_1))[2]
				adv_index_2 = findmin(abs(visited_s - adv_s_2))[2]
				visited_e_ys = agent.trajectories_s[i, index_closest_state : index_closest_state + 2 * horizon - 1, 2]
				visited_e_ys = visited_e_ys[:, adv_index_1 : adv_index_2, :]
				mean_e_y = mean(visited_e_ys)

				tolerance = 0.0
				
				if overtake_on_left
					if any(visited_e_ys .> interval[2] + tolerance)
						continue
					end
				else
					if any(visited_e_ys .< interval[1] - tolerance)
						continue
					end
				end

				if possible_trajectory_list == nothing
					 possible_trajectory_list = [i]
					 avg_e_y = [mean_e_y]
				else 
					append!(possible_trajectory_list, [i])
					append!(avg_e_y, [mean_e_y])
				end
			end
		end
	end

	if avg_e_y == nothing
		# select_states(agent)
		# println("STANDARD SELECTION")
		laps_per_init = round(Int64,round(Int64, NUM_LOADED_LAPS - 5) / 3)
		distance_to_pf = abs([EY_INNER, EY_CENTER, EY_OUTER] - adv_e_y)
		selected_init = findmax(distance_to_pf)[2]

		if selected_init == 1
			selected_laps = 5 + 1 * laps_per_init + collect(1 : NUM_CONSIDERED_LAPS)
		elseif selected_init == 2
			selected_laps = 5 + 0 * laps_per_init + collect(1 : NUM_CONSIDERED_LAPS)
		elseif selected_init == 3
			selected_laps = 5 + 2 * laps_per_init + collect(1 : NUM_CONSIDERED_LAPS)
		end

		# selected_laps = [2, 3, 4, 2]
		println("CONSERVATIVE SELECTION")
	else
		println("SMART SELECTION")
		num_possible_trajectories = length(possible_trajectory_list)

		sorted_iterations = sortperm(needed_iteration_array)
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
	end

	#=
	sorted_e_y = sortperm(avg_e_y)
	min_max_e_y = [possible_trajectory_list[sorted_e_y[1]]; 
				   possible_trajectory_list[sorted_e_y[end]]]

	if num_possible_trajectories >= NUM_CONSIDERED_LAPS
		if overtake_on_left
			possible_trajectory_list = possible_trajectory_list[sorted_e_y[1 : NUM_CONSIDERED_LAPS - 1]]
		else 
			possible_trajectory_list = possible_trajectory_list[sorted_e_y[end : - 1 : end - (NUM_CONSIDERED_LAPS - 2)]]
		end
	else
		possible_trajectory_list = possible_trajectory_list[randperm(num_possible_trajectories)]
	end

	while length(possible_trajectory_list) < NUM_CONSIDERED_LAPS
		append!(possible_trajectory_list, [possible_trajectory_list[randperm(length(possible_trajectory_list))[1]]])
	end

	=#

	# append!(possible_trajectory_list, [findmin(agent.iterations_needed)[2]])
	min_iteration = round(Int64, findmin(agent.iterations_needed[selected_laps])[1])

	println("selection: ", selected_laps)

	counter = 1
	iteration_number = 1
	for i in selected_laps
		agent.selected_laps[iteration_number] = i
		println("Closest index: ", closest_index)
		agent.selected_states_s[counter : counter + round(Int64, NUM_HORIZONS * horizon) - 1, :] = 
			squeeze(agent.trajectories_s[i, 
					closest_index[i] : closest_index[i] + round(Int64, NUM_HORIZONS * horizon) - 1, :], 
					1)
		agent.selected_states_xy[counter : counter + round(Int64, NUM_HORIZONS * horizon) - 1, :] = 
			squeeze(agent.trajectories_xy[i, 
					closest_index[i] : closest_index[i] + round(Int64, NUM_HORIZONS * horizon) - 1, :], 
					1)		
		#=
		agent.selected_states_cost[counter : counter + round(Int64, NUM_HORIZONS * horizon) - 1, :] = 
			collect((round(Int64, NUM_HORIZONS * horizon) : - 1 : 1) + agent.iterations_needed[i] - min_iteration) 
		=#

		agent.selected_states_cost[counter : counter + round(Int64, NUM_HORIZONS * horizon) - 1, :] = 
			collect(0 : - 1 : - (round(Int64, NUM_HORIZONS * horizon) - 1)) + agent.iterations_needed[i] - closest_index[i]
					
		agent.all_selected_states[agent.counter[end]][iteration_number] = (i, closest_index[i] : closest_index[i] + round(Int64, NUM_HORIZONS * horizon) - 1)

		counter += round(Int64, NUM_HORIZONS * horizon)
		iteration_number += 1		
	end

	agent.closest_indeces = closest_index[agent.selected_laps]

	for i = 1 : num_considered_states
		if abs(agent.selected_states_s[i, 2]) > TRACK_WIDTH / 2
			agent.selected_states_cost[i] += 20
		end
	end

	laps_for_sysid = selected_laps[1 : 2]
	identify_system!(agent, laps_for_sysid)
end

function identify_system!(agent::Agent, laps::Array{Int64})
	num_states = 50  # number of states used for system identification
	num_previous = SYS_ID_BEFORE
	num_after = SYS_ID_AFTER

	num_buffer = NUM_STATES_BUFFER

	# Create the current dynamic state: [v_x v_y psi_dot acc steering]
	current_iteration = agent.current_iteration

	# Mapping: [s, e_y, e_psi, psi_dot, v_x, v_y] --> [v_x v_y psi_dot]
	mapping = [5, 6, 4]
	# println(agent.states_s[current_iteration])
	# println(agent.current_input)
	current_s = agent.states_s[current_iteration, 1]
	v_x, v_y, psi_dot  = agent.states_s[current_iteration, mapping]

	# TODO: Add Delay
	# acc, steering = agent.current_input
	previous_lap = agent.current_lap - 1 + NUM_LOADED_LAPS
	if current_iteration <= agent.delay_a
		acc = agent.previous_inputs[previous_lap, agent.iterations_needed[previous_lap] - agent.delay_a + current_iteration + num_buffer + 1, 1]
	else
		acc = agent.inputs[current_iteration - agent.delay_a, 1]
	end
	if current_iteration <= agent.delay_df
		steering = agent.previous_inputs[previous_lap, agent.iterations_needed[previous_lap] - agent.delay_df + current_iteration + num_buffer + 1, 2]
	else
		steering = agent.inputs[current_iteration - agent.delay_df, 2]
	end

	current_dynamics = [v_x, v_y, psi_dot, acc, steering]
	# current_dynamics = squeeze(agent.dynamics[agent.num_sessions, max(agent.counter[agent.current_session] - 1, 1), :], (1, 2))
	# println("Current dynamics: ", current_dynamics)

	num_recorded_laps = agent.num_loaded_laps + agent.current_lap - 1
	# num_considered_states = size(agent.selected_states_s)[1]

	# Count the number of recorded states
	total_recorded_states = current_iteration - 1

	# Create the dynamics of all the recorded laps
	for i = 1 : num_recorded_laps
		total_recorded_states += agent.iterations_needed[i]
	end

	summed_iterations = cumsum([1; agent.iterations_needed])

	#######################################################################
	# Selection based on previous lap # 
	#######################################################################
	# Find closest states in terms of s
	@assert NUM_LOADED_LAPS > 1

	selected_dynamics = []

	println("LAPS: ", laps)

	for lap in laps
		println("LAP: ", lap)
		# previous_lap = agent.current_lap - 1 + NUM_LOADED_LAPS

		if lap in agent.not_for_dynamics
			lap = 2  # is always a complete path_following lap
		end

		println("iterations needed: ", agent.iterations_needed[lap])

		previous_indeces = (1 : agent.iterations_needed[lap]) + num_buffer
		previous_s = agent.trajectories_s[lap, previous_indeces, 1]

		distance_to_s = colwise(Euclidean(), previous_s, [current_s])
		closest_index = findmin(distance_to_s)[2]
		selected_indeces = collect((closest_index - num_previous : closest_index + num_after) + num_buffer)

		dynamic_states = squeeze(agent.trajectories_s[lap, :, mapping], 1)
		previous_acc = squeeze(agent.previous_inputs[lap, :, 1], 1)
		previous_acc = [zeros(agent.delay_a); previous_acc[1 : end - agent.delay_a]]
		previous_steering = squeeze(agent.previous_inputs[lap, :, 2], 1)
		previous_steering = [zeros(agent.delay_df); previous_steering[1 : end - agent.delay_df]]

		previous_inputs = [previous_acc previous_steering]
		dynamic_states = [dynamic_states previous_inputs]

		if size(selected_dynamics, 1) == 0
			selected_dynamics = dynamic_states[selected_indeces, :]
			next_dynamics = dynamic_states[selected_indeces + 1, 1 : 3]
		else
			selected_dynamics = [selected_dynamics; dynamic_states[selected_indeces, :]]
			next_dynamics = [next_dynamics; dynamic_states[selected_indeces + 1, 1 : 3]]
		end
	end

	current_lap = agent.current_lap + NUM_LOADED_LAPS
	previous_iteration = agent.current_iteration - 1

	if current_lap in agent.not_for_dynamics
		current_lap = 2  # is always a complete path_following lap
	end

	if false  # using current lap for system identification or not
		indeces = collect((max(1, previous_iteration - SYS_ID_AFTER) : previous_iteration - 1) + num_buffer)
		
		previous_dynamic_states = squeeze(agent.trajectories_s[current_lap, :, mapping], 1)
		previous_acc = squeeze(agent.previous_inputs[current_lap, :, 1], 1)
		previous_acc = [zeros(agent.delay_a); previous_acc[1 : end - agent.delay_a]]
		previous_steering = squeeze(agent.previous_inputs[current_lap, :, 2], 1)
		previous_steering = [zeros(agent.delay_df); previous_steering[1 : end - agent.delay_df]]

		previous_inputs = [previous_acc previous_steering]
		previous_dynamic_states = [previous_dynamic_states previous_inputs]

		selected_dynamics = [selected_dynamics; previous_dynamic_states[indeces, :]]
		next_dynamics = [next_dynamics; previous_dynamic_states[indeces + 1, 1 : 3]]
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

	if any(isnan(dummy_vy)) || any(isnan(dummy_psi_dot))
		println("Not updating thetas.")
		println("selected_dynamics: ", selected_dynamics)
		exit()
	else
		agent.theta_vx = dummy_vx
		agent.theta_vy = dummy_vy
		agent.theta_psi_dot = dummy_psi_dot
	end

	# println("THETA VX: ", agent.theta_vx)
	# println("THETA VY: ", agent.theta_vy)
	# println("THETA PSI DOT: ", agent.theta_psi_dot)
end

function determine_needed_iterations!(agent::Agent, indeces)
	num_buffer = NUM_STATES_BUFFER

    println(agent.iterations_needed)

	for i = indeces
		agent.iterations_needed[i] = findmin(sumabs(agent.trajectories_s[i, :, :], (1, 3)))[2] - 1 - 2 * num_buffer
		# No data has been appended to the last lap yet.
		if i == indeces[end]
			agent.iterations_needed[i] += num_buffer
		end
		# No data has been appended to the first lap yet.
		if i == indeces[1]
			agent.iterations_needed[i] += num_buffer
			if agent.iterations_needed[i] < 1
				agent.iterations_needed[i] = findmin(sumabs(agent.trajectories_s[i, num_buffer + 1 : end, :], (1, 3)))[2] - 1 - num_buffer
			end
		end
	end
end

function determine_needed_iterations!(agent::Agent)
	determine_needed_iterations!(agent, 1 : agent.num_loaded_laps)
end