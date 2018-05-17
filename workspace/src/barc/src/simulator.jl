#=
	File name: simulator.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

# addprocs(2)
# using DistributedArrays

include("track.jl")
include("agent.jl")
include("optimizer.jl")
include("plotter.jl")


type Simulator
	num_laps::Int64
	current_lap::Int64
	maximum_num_iterations::Int64
	num_loaded_laps::Int64

	current_iteration::Int64

	track::Track

	num_agents::Int64
	agents::Array{Agent}
	optimizers::Array{Optimizer}
	horizon::Int64
	references::Array{Float64}
	current_reference::Array{Float64}

	learning::Bool
	elapsed_time::Float64

	plotter::Plotter

	Simulator() = new()
end

function init!(simulator::Simulator, num_laps::Int64, num_agents::Int64, 
			   horizon::Int64, references::Array{Float64}, 
			   num_considered_states::Int64, num_loaded_laps::Int64)
	elapsed_time = 0.0  # simulation time, not actual time
	simulator.learning = false
	simulator.num_agents = num_agents
	simulator.num_laps = num_laps  # only simulate one lap at first
	simulator.maximum_num_iterations = 2000
	simulator.current_lap = 1
	# simulator.num_iterations_needed = zeros(simulator.num_laps)
	simulator.horizon = horizon
	simulator.references = references
	simulator.current_reference = simulator.references[1, :]

	init_states = INIT_STATES
	v_max = V_MAX
	color = COLOR

	simulator.track = Track()
	init!(simulator.track)
	# load_track!(simulator.track, "some_filename.jld")

	simulator.num_loaded_laps = num_loaded_laps
	# println("num loaded laps2: ", num_loaded_laps)

	# initialize agents
	agents = Array{Agent}(simulator.num_agents)
	optimizers = Array{Optimizer}(simulator.num_agents)
	for i = 1 : simulator.num_agents
		agents[i] = Agent()
		init!(agents[i], i, simulator.track,
			  simulator.maximum_num_iterations, simulator.horizon, 
			  simulator.num_laps, num_considered_states, num_loaded_laps, 
			  v_max[i], color[i])
		set_state_s!(agents[i], init_states[i, :], simulator.track)
	end
	for i = 1 : simulator.num_agents
		optimizers[i] = Optimizer()
		if simulator.num_agents > 1
			agent_indeces = collect(1 : simulator.num_agents)
			deleteat!(agent_indeces, agents[i].index)
			adversarial_agents = agents[agent_indeces]
			init!(optimizers[i], agents[i], simulator.horizon, 
				  adversarial_agents)
			try 
				optimizers[i].adversarial_agents
				if simulator.num_agents > 2
					println("There are ", 
							length(optimizers[i].adversarial_agents), 
							" adversarial agents")
				else
					println("There is 1 adversarial agent")
				end
			catch
				println("No adversarial agents")
			end
		else
			init!(optimizers[i], agents[i], simulator.horizon)
		end
		if simulator.learning
			init_learning_mpc!(optimizers[i], simulator.track)
		else
			init_path_following!(optimizers[i], simulator.track)
		end
	end

	# turn agent and optimizer arrays into distributed arrays
	simulator.agents = agents
	simulator.optimizers = optimizers

	simulator.plotter = Plotter()
	init!(simulator.plotter, simulator.track, simulator.agents)
end

function simulate_iteration!(simulator::Simulator)
	# TODO: multiple agents
	# TODO: parallelize execution of multiple agents
	# TODO: switch between path following and learning mpc

	distance = nothing

	for i = 1 : simulator.num_agents
		propagate_adv_prediction!(simulator.optimizers[i])
	end

	for i = 1 : simulator.num_agents
		leading = false
		println("I am solving agent $i")
		optimizer = simulator.optimizers[i]
		agent = simulator.agents[i]
		current_iteration = agent.current_iteration

		# reset the weights
		num_considered_states = size(simulator.agents[i].selected_states_s)[1]
		simulator.agents[i].weights_states = ones(num_considered_states)

		agent.predicted_xy = zeros(agent.predicted_xy)

		if simulator.learning
			# println("num loaded laps 3: ", simulator.num_loaded_laps)
			if simulator.num_agents > 1
				distance = get_distance_on_track(simulator)
				# println("DISTANCE: ", distance)
				# if get_total_distance(simulator.optimizers[i], simulator.track.total_length) >= 0
				# println("direct distance on track: ", get_leading_agent_on_track(simulator, i))
				if get_leading_agent_on_track(simulator, i) >= 0
					leading = true
				end
			end

			if simulator.num_agents == 1 
				# println("HERE")
				# chosen_lap = simulator.num_loaded_laps + agent.current_lap - 1
				chosen_lap = simulator.num_loaded_laps
				select_states(agent)
				# states_cost(agent, chosen_lap)
			elseif distance > 2 * simulator.horizon * simulator.track.ds || leading
				chosen_lap = simulator.num_loaded_laps
				# chosen_lap = simulator.num_loaded_laps + agent.current_lap - 1
				select_states(agent)
				# states_cost(agent, chosen_lap)
			else	
				adv_agent = optimizer.adversarial_agents[1]
				adv_agent_iter = adv_agent.current_iteration
				adv_agent_e_y = adv_agent.states_s[adv_agent_iter, 2]
				# adv_agent_e_y = adv_agent.predicted_s[end, 2]
				adv_agent_v = adv_agent.states_s[adv_agent_iter, 4]
				if adv_agent_v > agent.states_s[current_iteration, 4]
					select_states(agent)
				else
					select_states_smartly(agent, adv_agent_e_y, simulator.track)
					adjust_convex_hull(optimizer)
				end
			end

			# println(optimizer.agent.iterations_needed)

			# println(agent.selected_states_cost)
			tic()
			solve_learning_mpc!(optimizer, simulator.track, leading)
			toc()
		else
			solve_path_following!(optimizer, simulator.track, 
							  	  simulator.current_reference)
		end

		# determines the optimal inputs and also states in s 
		agent.optimal_inputs = optimizer.solution_inputs
		
		# TODO: I guess predicted_(s or xy) is a bad variablename,
		# since they are not the actual predictions
		# agent.predicted_s = optimizer.solution_states_s  # the predicted s states, not the actual ones
		agent.predicted_s = simulate_s_kin(agent, simulator.track, 
		 								   agent.optimal_inputs)  # true s states
		# agent.predicted_s = optimizer.solution_states_s

		predicted_true_s = simulate_s_dyn(agent, simulator.track, 
										  agent.optimal_inputs)

		# println("predicted true s: ", predicted_true_s)

		# println("Size of predicted_s: ", size(agent.predicted_s))
		agent.predicted_xy = simulate_xy(agent, agent.optimal_inputs)  # true xy states
		# println("BEFORE ", agent.predicted_xy)

		#=
		for j = 1 : HORIZON + 1
			agent.predicted_xy[j, :] = s_to_xy(simulator.track, agent.predicted_s[j, :], agent)
		end
		=#
		for j = 1 : HORIZON + 1
			agent.predicted_true_xy[j, :] = s_to_xy(simulator.track, 
													true_s_to_s(predicted_true_s[j, :]), 
													agent)
			agent.predicted_xy[j, :] = s_to_xy(simulator.track, 
											   agent.predicted_s[j, :], agent)
		end

		xy_final = xy_to_s(simulator.track, agent.predicted_xy[end, :])
		println("................: ", optimizer.solution_states_s[end, :])
		println("................: ", predicted_true_s[end, :])
		println("................: ", true_s_to_s(predicted_true_s[end, :]))
		println("CHECK IF OUTSIDE: ", xy_final)

		println("True S: ", predicted_true_s[:, 1])
		println("sim S: ", agent.predicted_s[:, 1])

		if abs(xy_final[2]) > simulator.track.width / 2
			warn("OUTSIDE")
			# sleep(5)
		end

		

		# println("AFTER ", agent.predicted_xy)


		# println("predicted_s: ", agent.predicted_s)
		# println("plength pred s: ", size(agent.predicted_s))

		# apply the first input and update the states accordingly
		agent.inputs[current_iteration, :] = agent.optimal_inputs[1, :]
		agent.current_input = agent.optimal_inputs[1, :]
		println("optimal inputs:", agent.optimal_inputs[1, :])

		
		# BETTER RESULTS
		agent.states_s[current_iteration + 1, :] = agent.predicted_s[2, :]
		# this gives better results for the states
		agent.states_xy[current_iteration + 1, :] = s_to_xy(simulator.track, 
		 											agent.predicted_s[2, :], 
		 											agent)

		# TODO: Looks like there is still an error for xy_to_s(...)
		# agent.states_xy[current_iteration + 1, :] = agent.predicted_xy[2, :]
		# agent.states_s[current_iteration + 1, :] = xy_to_s(simulator.track, agent.predicted_xy[2, :])

		# println(agent.predicted_s)
		# println(predicted_true_s)
		agent.states_true_s[current_iteration + 1, :] = predicted_true_s[2, :]
		true_s = agent.states_true_s[current_iteration + 1, :]
		agent.states_s[current_iteration + 1, :] = true_s_to_s(true_s)
		agent.states_xy[current_iteration + 1, :] = true_s_to_xy(simulator.track, 
		 														 true_s, agent)
		
		#=
		test = deepcopy(agent)
		test_track = deepcopy(simulator.track)
		for i = 1 : HORIZON + 1
			agent.predicted_xy[i, :] = s_to_xy(test_track, test.predicted_s[i, :], test)
		end

		println("AFTER: ", test.predicted_xy)
		=#

		#=
		# println(predicted_true_s)
		# println(agent.predicted_s)
		for i = 1 : HORIZON + 1
			dummy = agent.predicted_s[i, :]
			agent.predicted_xy[i, :] = s_to_xy(simulator.track, dummy, agent)
		end
		# println(agent.predicted_xy)
		=#

		println("current state s: ", agent.states_s[current_iteration, :])
		println("current true  s: ", agent.states_true_s[current_iteration, :])
		# println("predicted state s: ", agent.predicted_s)
		# println("iteration1: ", agent.current_iteration)
		println("current state xy: ", agent.states_xy[current_iteration, :])

		# update adversarial agents
		for j = 1 : simulator.num_agents
			if j == i 
				continue
			else
				# TODO: for more than two agents
				simulator.optimizers[j].adversarial_agents[1].predicted_s = 
					agent.predicted_s
			end
		end

		# set_state_s!(agent, agent.states_s[current_iteration, :], simulator.track)
		# sim_s = simulate_s_kin(agent, simulator.track, agent.optimal_inputs)  # true s states
		# println("simulated s: ", sim_s)
		# println("length sim s: ", size(sim_s))
		# println("simulated s:", simulate_s_kin(agent, simulator.track, agent.optimal_inputs[1, :]))
	end
end

function simulate_lap!(simulator::Simulator)
	leading_index = 0
	following_index = 0

	# Set all the agents current iteration to 1
	for i = 1 : simulator.num_agents
		if simulator.current_lap == 1
			simulator.agents[i].current_iteration = 1
		end

		agent = simulator.agents[i]
		# Initialize the new lap with the states and inputs from the the end
		# of the previous lap
		if simulator.current_lap > 1
			current_iteration = simulator.agents[i].current_iteration
		end	
	end

	agent = simulator.agents[1]
	track = simulator.track

	iteration = 1
	finished = false

	while iteration <= simulator.maximum_num_iterations && !finished
		println("Iteration: ", iteration)
		simulate_iteration!(simulator)
		# simulate_iteration_parallel!(simulator)

		if PLOTTING_2 
			update!(simulator.plotter, simulator.agents)
			plot_race_info!(simulator.plotter, simulator.agents,
			     		    simulator.elapsed_time, simulator.num_laps)
		end

		if PLOTTING
			plt[:clf]()
			# plot_track(track)
			# plot_finish_line(track)
			

			for i = 1 : simulator.num_agents
				agent = simulator.agents[i]
				plot_agent(agent, agent.current_iteration)
			end

			if FOCUSED_PLOT
				if simulator.num_agents < 2
					center_of_mass = simulator.agents[1].states_xy[simulator.agents[1].current_iteration, 1:2]
				else
					center_of_mass = simulator.agents[2].states_xy[simulator.agents[2].current_iteration, 1:2]
				end
				# plt[:xlim]([center_of_mass[1] - 0.75, center_of_mass[1] + 0.75])
				# plt[:ylim]([center_of_mass[2] - 0.75, center_of_mass[2] + 0.75])
				plt[:xlim]([center_of_mass[1] - 1.5, center_of_mass[1] + 1.5])
				plt[:ylim]([center_of_mass[2] - 1.5, center_of_mass[2] + 1.5])
			end

			# plt[:text](2, 4,"A test string")
			# plt[:figtext](0.5, 0,"Comment: Test string that is a note about something", wrap=true, horizontalalignment="center", fontsize=12)

			##plt[:figure](figsize=(1920, 1081), dpi=100)
			
			if SAVE_PLOTS
				fig = plt[:gcf]()
				fig[:set_size_inches]((19.2, 10.81), forward=false)
				# TODO: change directory
				fig[:savefig]("/media/lukas/TOSHIBA EXT/master_thesis/images/track_3_dyn/iteration_$(simulator.current_lap)_$(simulator.current_iteration).jpg", dpi=500)
			end
			## plt[:savefig]("test.jpg")
			fig = plt[:gcf]()
			fig[:canvas][:set_window_title]("Racing Simulation")

			plt[:show]()
			plt[:pause](0.000001)
		end

		for i = 1 : simulator.num_agents
			agent = simulator.agents[i]
			current_iteration = agent.current_iteration

			println("Current Iteration: ", current_iteration)

			simulator.agents[i].final_s = agent.states_s[current_iteration + 1, :]
			final_true_s = agent.states_true_s[current_iteration + 1, :]
			simulator.agents[i].final_xy = agent.states_xy[current_iteration + 1, :]
			simulator.agents[i].final_input = agent.inputs[current_iteration + 1, :]

			# if agent.states_s[current_iteration + 1, 1] >= track.total_length
			if agent.states_true_s[current_iteration + 1, 1] >= track.total_length			
				s_coords = agent.states_s[current_iteration + 1, :]
				simulator.agents[i].states_s[current_iteration + 1, 1] = get_s_coord(track, 
															   s_coords[1])
				simulator.agents[i].final_s = simulator.agents[i].states_s[current_iteration + 1, :]

				true_s_coords = agent.states_true_s[current_iteration + 1, 1]
				simulator.agents[i].states_true_s[current_iteration + 1, 1] = get_s_coord(track, true_s_coords[1])
				final_true_s = simulator.agents[i].states_true_s[current_iteration + 1, :]

				if simulator.agents[i].current_lap + 1 > simulator.current_lap
					finished = true
					simulator.current_lap += 1
				end

				current_lap = simulator.agents[i].current_lap
				println("Current Lap: ", current_lap)

				if current_lap > 1 || MODE == "path_following"
					if MODE == "path_following"
						current_lap += 1
					end
					# TODO: Check if this actually works
					simulator.agents[i].iterations_needed[current_lap - 1 + 
											simulator.num_loaded_laps] = agent.current_iteration
					# println("Num loaded laps: ", simulator.num_loaded_laps)
					needed_iterations = simulator.agents[i].iterations_needed[current_lap - 1 + simulator.num_loaded_laps]

					simulator.agents[i].trajectories_s[current_lap - 1 + simulator.num_loaded_laps, :, :] = simulator.agents[i].states_s
					simulator.agents[i].trajectories_xy[current_lap - 1 + simulator.num_loaded_laps, :, :] = simulator.agents[i].states_xy

					track_length = simulator.track.total_length
					num_considered_states = size(simulator.agents[i].selected_states_s)[1]

					simulator.agents[i].trajectories_s[current_lap - 1 + simulator.num_loaded_laps, needed_iterations + 1 : needed_iterations + num_considered_states, :] = simulator.agents[i].states_s[1 : num_considered_states, :] + ([track_length 0 0 0] .* ones(num_considered_states, 4))
					simulator.agents[i].trajectories_xy[current_lap - 1 + simulator.num_loaded_laps, needed_iterations + 1 : needed_iterations + num_considered_states, :] = simulator.agents[i].states_xy[1 : num_considered_states, :] 
				end
				# println(simulator.agents[i].trajectories_s[current_lap + simulator.num_loaded_laps, :, 1])

				# reset states and inputs
				simulator.agents[i].states_true_s = zeros(agent.states_true_s)
				simulator.agents[i].states_s = zeros(agent.states_s)
				simulator.agents[i].states_xy = zeros(agent.states_xy)
				simulator.agents[i].inputs = zeros(agent.inputs)

				# println(simulator.agents[i].states_true_s[current_iteration : current_iteration + 1, 1])

				# initialize states and inputs
				simulator.agents[i].states_s[1, :] = simulator.agents[i].final_s
				simulator.agents[i].states_true_s[1, :] = final_true_s
				simulator.agents[i].states_xy[1, :] = simulator.agents[i].final_xy
				simulator.agents[i].inputs[1, :] = simulator.agents[i].final_input

				simulator.agents[i].current_lap += 1
				# setting iteration to 0 so that the iteration is set to 
				# 1 in the following loop
				simulator.agents[i].current_iteration = 0 
			end
		end

		for i = 1 : simulator.num_agents
			simulator.agents[i].current_iteration += 1
		end

		iteration += 1
		simulator.current_iteration = iteration
		simulator.elapsed_time += simulator.agents[1].dt

	end

	println("Finished lap")
end

function simulate_race!(simulator::Simulator)
	for i = 1 : simulator.num_agents
		if simulator.learning
			init_learning_mpc!(simulator.optimizers[i], simulator.track)
		else
			init_path_following!(simulator.optimizers[i], simulator.track)
		end
	end
	main_agent = simulator.agents[1]

	# TODO
	for i = 1:simulator.num_laps
		if !simulator.learning
			simulator.current_reference = simulator.references[i, :]
		end
		println("Lap ", i)
		simulate_lap!(simulator)

		#=
		main_agent.trajectories_s[i + simulator.num_loaded_laps, :, :] = main_agent.states_s
		main_agent.trajectories_xy[i + simulator.num_loaded_laps, :, :] = main_agent.states_xy

		needed_iterations = main_agent.iterations_needed[i + simulator.num_loaded_laps]

		track_length = simulator.track.total_length
		num_considered_states = size(main_agent.selected_states_s)[1]

		main_agent.trajectories_s[i + simulator.num_loaded_laps, needed_iterations + 1 : needed_iterations + num_considered_states, :] = main_agent.states_s[1 : num_considered_states, :] + ([track_length 0 0 0] .* ones(num_considered_states, 4))
		main_agent.trajectories_xy[i + simulator.num_loaded_laps, needed_iterations + 1 : needed_iterations + num_considered_states, :] = main_agent.states_xy[1 : num_considered_states, :] 
		=#

		#=
		if simulator.num_laps == 2
			simulator.learning = true
		end
		=#
		# plot_trajectories(main_agent, simulator.track, collect(1 : (simulator.num_laps + simulator.num_loaded_laps))')
		# plot_trajectories(main_agent, simulator.track, [i + simulator.num_loaded_laps])
	end
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

function get_distance_on_track(simulator::Simulator)
	# calculate the distance on the track
	@assert length(simulator.agents) > 1
	track_length = simulator.track.total_length
	s_coords = zeros(length(simulator.agents))

	for i = 1 : simulator.num_agents
		iteration = simulator.agents[i].current_iteration
		s_coords[i] = simulator.agents[i].states_s[iteration, 1] 
	end 

	distance = abs(sum(s_coords .* [1; - 1]))

	if distance > track_length / 2
		distance = track_length - distance
	end

	return distance
end

function get_leading_agent_on_track(simulator::Simulator, idx::Int64)
	track_length = simulator.track.total_length
	s_coords = zeros(length(simulator.agents))

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

	# println("S-coordinates: ", s_coords)

	distance = sum(s_coords .* [1; - 1])

	return distance
end

function plot_race_info(simulator::Simulator)
	elapsed_time = simulator.elapsed_time
	time_string = "Time : $(round(elapsed_time, 2)) s"
	# plt[:annotate](time_string, xy=[0.5; 0.9], 
	plt[:annotate](time_string, xy=[0.5; 0.7], 
				   xycoords= "axes fraction", fontsize=20, 
				   verticalalignment="top")
	if length(simulator.agents) == 2
	    if simulator.agents[1].current_lap > simulator.agents[2].current_lap
			leading_index = 1
			following_index = 2
		elseif simulator.agents[1].current_lap == simulator.agents[2].current_lap
			if simulator.agents[1].states_s[simulator.agents[1].current_iteration, 1] >
			   simulator.agents[2].states_s[simulator.agents[2].current_iteration, 1]
			    leading_index = 1
			    following_index = 2
			else
				leading_index = 2
				following_index = 1
			end
		else
			leading_index = 2
			following_index = 1
		end

		leading_agent = ucfirst(simulator.agents[leading_index].color)
		leading_lap = simulator.agents[leading_index].current_lap
		following_agent = ucfirst(simulator.agents[following_index].color)
		following_lap = simulator.agents[following_index].current_lap

		race_string = @sprintf "%-10s %-10s\n%-10s %-10s\n%-10s %-10s" "Race" "Lap" "1. $(leading_agent)" "$(leading_lap)/$(simulator.num_laps)" "2. $(following_agent)" "$(following_lap)/$(simulator.num_laps)"
	else
		leading_agent = ucfirst(simulator.agents[1].color)
		leading_lap = simulator.agents[1].current_lap
		race_string = @sprintf "%-10s %-10s\n%-10s %-10s" "Race" "Lap" "1. $(leading_agent)" "$(leading_lap)/$(simulator.num_laps)"
	end
	# race_title = "Race    Lap\n1. $(leading_agent)   $(leading_lap)/$(simulator.num_laps)\n2. $(following_agent)   $(following_lap)/$(simulator.num_laps)"
	# plt[:annotate](race_string, xy=[0.8; 0.9], xycoords="axes fraction", 
	plt[:annotate](race_string, xy=[0.5; 0.5], xycoords="axes fraction", 
				   fontsize=20, verticalalignment="top")
end

#=
num_agents = 1

sim = Simulator()
init!(sim, num_agents, 10)

track = sim.track
=#

#=
plot(track.xy_coords[:, 1], track.xy_coords[:, 2], "k--")
plot(track.xy_outer[:, 1], track.xy_outer[:, 2], "k-")
plot(track.xy_inner[:, 1], track.xy_inner[:, 2], "k-")
=#

#=
xy_coord = [8.7, -1.5, 1.5, 0.1, 0.4, 0.1]
println(xy_coord)
steering_angle = 0.5
# xy_coord = [10.0, 0.5, 1.5, 0.1, 0.4, 0.1]
plot(xy_coord[1], xy_coord[2], "ro")
test = xy_to_s(track, xy_coord)
println(test)
test_2 = s_to_xy(track, test, sim.agents[1])
println(test_2)

s_coord = [9.3, 0.42, 1.1, 1.5]
println(s_coord)
test_3 = s_to_xy(track, s_coord, sim.agents[1])
println(test_3)
test_4 = xy_to_s(track, test_3)
println(test_4)

s_coord = [1, 0.5, -0.2, 1.5]
set_state_s!(sim.agents[1], s_coord, track)
#set_state_xy!(sim.agents[1], xy_coord, track)
sim.agents[1].current_input[2] = steering_angle
println(sim.agents[1].current_input)
iteration = 1
# plot_agent(sim.agents[1], track, iteration)
=#

#=
s_coord = [0.036502601857768456, -2.195692050625091e-5, -0.0001585575169261438, 0.24125652356563143]
#s_coord = [50.31, 0.54, 0.0, 0.0]
set_state_s!(sim.agents[1], s_coord, track)
println(s_coord)
steering_angle = 0.0
sim.agents[1].current_input[2] = steering_angle
test_2 = s_to_xy(track, s_coord, sim.agents[1])
println(test_2)
test = xy_to_s(track, test_2)
println(test)
iteration = 1
plot_agent(sim.agents[1], track, iteration)
=#

#=
# test simulation
horizon = 120
inputs = zeros(horizon, 2)
# inputs[:, 1] = 0.1 * ones(horizon, 1)
inputs[end:-1:1, 1] = cumsum(0.01 * ones(horizon, 1))
inputs[20:end, 2] = - 0.05 * ones(101, 1)

agent = sim.agents[1]
iteration = 1
states_xy = simulate_xy(agent, inputs, iteration)

plot(states_xy[:, 1], states_xy[:, 2], "ro")
=#

# axis("equal")
# grid("on")

#plt[:show]()
