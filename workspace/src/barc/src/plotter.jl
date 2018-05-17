#=
	File name: plotter.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

@pyimport matplotlib.patches as patch
@pyimport matplotlib.transforms as transforms
@pyimport matplotlib.animation as animation
@pyimport matplotlib as mpl
@pyimport matplotlib.collections as collections

# include("agent.jl")
include("track.jl")


type Plotter
	initialization::Bool
	ax::PyCall.PyObject

	front::Array{PyCall.PyObject}
	rear::Array{PyCall.PyObject}
	front_axis::Array{PyCall.PyObject}
	rear_axis::Array{PyCall.PyObject}

	rect::Array{PyCall.PyObject}
	front_wheel_left::Array{PyCall.PyObject}
	front_wheel_right::Array{PyCall.PyObject}
	rear_wheel_left::Array{PyCall.PyObject}
	rear_wheel_right::Array{PyCall.PyObject}

	selected_states::Array{PyCall.PyObject}
	sel_states_connected::Array{PyCall.PyObject}
	predicted_xy::Array{PyCall.PyObject}
	predicted_xy_con::Array{PyCall.PyObject}
	predicted_true_xy::Array{PyCall.PyObject}
	predicted_true_xy_con::Array{PyCall.PyObject}
	trajectory::Array{PyCall.PyObject}
	center_of_mass::Array{PyCall.PyObject}
	deleted_states::Array{PyCall.PyObject}

	time_s::PyCall.PyObject
	race_s::PyCall.PyObject

	com::Array{Float64}  # center of mass
	counter::Int64

	Plotter() = new()
end

function init!(plotter::Plotter, track::Track, agents::Array{Agent})
	plotter.initialization = true

	figure("Racing Simulation")
	# mng = plt[:get_current_fig_manager]()
	# mng[:full_screen_toggle]()

	plotter.ax = axes()
	plotter.ax[:set_aspect]("equal")

	plot_track!(plotter, track)
	plot_finish_line!(plotter, track)

	plotter.front = Array{PyCall.PyObject}(1)
	plotter.rear = Array{PyCall.PyObject}(1)
	plotter.front_axis = Array{PyCall.PyObject}(1)
	plotter.rear_axis = Array{PyCall.PyObject}(1)

	plotter.rect = Array{PyCall.PyObject}(1)
	plotter.front_wheel_left = Array{PyCall.PyObject}(1)
	plotter.front_wheel_right = Array{PyCall.PyObject}(1)
	plotter.rear_wheel_left = Array{PyCall.PyObject}(1)
	plotter.rear_wheel_right = Array{PyCall.PyObject}(1)

	plotter.selected_states = Array{PyCall.PyObject}(1)
	plotter.sel_states_connected = Array{PyCall.PyObject}(1)
	plotter.predicted_xy = Array{PyCall.PyObject}(1)
	plotter.predicted_xy_con = Array{PyCall.PyObject}(1)
	plotter.predicted_true_xy = Array{PyCall.PyObject}(1)
	plotter.predicted_true_xy_con = Array{PyCall.PyObject}(1)
	plotter.trajectory = Array{PyCall.PyObject}(1)
	plotter.center_of_mass = Array{PyCall.PyObject}(1)
	plotter.deleted_states = Array{PyCall.PyObject}(1, 
													length(agents[1].selected_states_xy))

	plotter.com = zeros(1, 2)  # center of mass

	iteration = 1
	for i = 1 : length(1)
		plot_agent!(plotter, agents[i], iteration)
	end

	plot_race_info!(plotter, agents, 0.0, 0)

	plotter.counter = 0

	plotter.initialization = false
end

function update!(plotter::Plotter, agents::Array{Agent}, iteration)
	for i = 1 : 1
		plot_agent!(plotter, agents[i], iteration)
	end

	if FOCUSED_PLOT
		plt[:xlim]([plotter.com[1, 1] - 1.5, 
					plotter.com[1, 1] + 1.5])
		plt[:ylim]([plotter.com[1, 2] - 1.5, 
					plotter.com[1, 2] + 1.5])
	end

	if SAVE_PLOTS
		plotter.counter += 1
		fig = plt[:gcf]()
		fig[:set_size_inches]((19.2, 10.81), forward=false)
		# TODO: change directory
		fig[:savefig]("/media/lukas/TOSHIBA EXT/master_thesis/images/ugo_slides/iteration_$(plotter.counter).jpg", dpi=500)
	end

end

plt[:ion]()
# cfig = figure(1)

function plot_track!(plotter::Plotter, track::Track)
	plotter.ax[:plot](track.xy_coords[:, 1], track.xy_coords[:, 2], "k--")
	plotter.ax[:plot](track.xy_outer[:, 1], track.xy_outer[:, 2], "k-")
	plotter.ax[:plot](track.xy_inner[:, 1], track.xy_inner[:, 2], "k-")

	axis("equal")
	# grid("on")
end

function plot_finish_line!(plotter::Plotter, track::Track)
	plotter.ax[:plot]([0; 0], [track.width / 2; - track.width / 2], 
					  linewidth=5.0, "k--")
end


function plot_agent!(plotter::Plotter, agent::Agent, iteration::Int64)
	ax_plot = plotter.ax[:plot]

	index = agent.index

	l_front = agent.l_front
	l_rear = agent.l_rear
	# TODO: agent's COM assumed to be in the geometric center 
	agent_length = l_front + l_rear 
	width = agent.width
	psi = agent.states_xy[iteration, 5]
	psi_degrees = psi * 180 / pi
	psi_and_steering = psi + agent.current_input[2]

	center_of_mass = agent.states_xy[iteration, 1:2]

	plotter.com[agent.index, :] = center_of_mass

	front = center_of_mass + rotate_around_z([l_front 0]', psi)'
	rear = center_of_mass - rotate_around_z([l_rear 0]', psi)'

	# ax = cfig[:add_subplot](1,1,1)
	
	wheel_length = 0.04
	wheel_width = 0.02

	center_front_wheel = [0 (width / 2); 0 (- width / 2)]
	center_front_wheel = (rotate_around_z(center_front_wheel', psi))'
	center_front_wheel = repmat(front, 2) + center_front_wheel
	center_rear_wheel = [0 (width / 2); 0 (- width / 2)]
	center_rear_wheel = (rotate_around_z(center_rear_wheel', psi))'
	center_rear_wheel = repmat(rear, 2) + center_rear_wheel

	if plotter.initialization
		plotter.front[index] = ax_plot([center_of_mass[1] front[1]]', 
									   [center_of_mass[2] front[2]]', 
 									   linewidth=3.0, "k-")[1]

		plotter.rear[index] = ax_plot([center_of_mass[1] rear[1]]', 
									  [center_of_mass[2] rear[2]]', 
			 						  linewidth=3.0,  "k-")[1]
		plotter.front_axis[index] = ax_plot(center_front_wheel[:, 1], 
											center_front_wheel[:, 2], 
							 				linewidth=3.0, "k-")[1]
		plotter.rear_axis[index] = ax_plot(center_rear_wheel[:, 1], 
										   center_rear_wheel[:, 2], 
										   linewidth=3.0, "k-")[1]
	else
		plot_front = plotter.front[index]
		plot_rear = plotter.rear[index]
		plot_front_axis = plotter.front_axis[index]
		plot_rear_axis = plotter.rear_axis[index]

		plot_front[:set_data]([center_of_mass[1] front[1]]', 
							  [center_of_mass[2] front[2]]')
		plot_rear[:set_data]([center_of_mass[1] rear[1]]', 
							 [center_of_mass[2] rear[2]]')
		plot_front_axis[:set_data](center_front_wheel[:, 1], 
								   center_front_wheel[:, 2])
		plot_rear_axis[:set_data](center_rear_wheel[:, 1], 
								  center_rear_wheel[:, 2])
	end

	if agent.color == "blue"
		agent_dot = "bo"
		agent_star = "b*"
		agent_line = "b-"
		agent_dot_extra = "co"
		agent_line_extra = "c-"
	elseif agent.color == "red"
		agent_dot = "ro"
		agent_star = "r*"
		agent_line = "r-"
		agent_dot_extra = "mo"
		agent_line_extra = "m-"
	else 
		error("Color $(agent.color) is not defined")
	end

	if !CLEAN_PLOT
		ellipse = patch.Ellipse([center_of_mass[1], center_of_mass[2]], 
								2 * agent_length, 2 * agent.width, 
								angle=psi_degrees, fc="gray")
		# ax[:add_artist](ellipse)

		if plotter.initialization
			plotter.selected_states[index] = ax_plot(agent.selected_states_xy[:, 1], 
									  		  		 agent.selected_states_xy[:, 2], 
									  		  		 agent_dot_extra)[1]
		else
			plot_selection = plotter.selected_states[index]
			plot_selection[:set_data](agent.selected_states_xy[:, 1], 
									  agent.selected_states_xy[:, 2])
		end

		horizon = size(agent.predicted_xy)[1] - 1
		num_considered_laps = round(Int64, size(agent.selected_states_xy)[1] / 
									  (2 * horizon))

		# println("considered laps: ", num_considered_laps)
		for i = 1 : num_considered_laps
			start_idx = (i - 1) * 2 * horizon + 1
			end_idx = start_idx + 2 * horizon - 1
			# println(start_idx, " : ", end_idx)
			if plotter.initialization
				plotter.sel_states_connected[index] = ax_plot(agent.selected_states_xy[start_idx : end_idx, 1], 
											   		   		  agent.selected_states_xy[start_idx : end_idx, 2], 
											   		   		  agent_line_extra)[1]
			else
				plotter.sel_states_connected[index][:set_data](agent.selected_states_xy[start_idx : end_idx, 1], 
											    	    	   agent.selected_states_xy[start_idx : end_idx, 2])
			end
		end

		if plotter.initialization
			plotter.predicted_xy[index] = ax_plot(agent.predicted_xy[:, 1], 
										   		  agent.predicted_xy[:, 2], 
										   		  agent_dot)[1]
			plotter.predicted_xy_con[index] = ax_plot(agent.predicted_xy[:, 1], 
											   		  agent.predicted_xy[:, 2], 
											   		  agent_line)[1]
			plotter.predicted_true_xy[index] = ax_plot(agent.predicted_true_xy[:, 1], 
										   		  agent.predicted_true_xy[:, 2], 
										   		  agent_star)[1]
			plotter.predicted_true_xy_con[index] = ax_plot(agent.predicted_true_xy[:, 1], 
											   		  agent.predicted_true_xy[:, 2], 
											   		  agent_line)[1]
			plotter.trajectory[index] = ax_plot(agent.states_xy[1 : agent.current_iteration, 1], 
			     						 		agent.states_xy[1 : agent.current_iteration, 2], 
			     						 		agent_line)[1]
			plotter.center_of_mass[index] = ax_plot(center_of_mass[1], 
													center_of_mass[2], "ko")[1]
		else
			plotter.predicted_xy[index][:set_data](agent.predicted_xy[:, 1], 
										   		   agent.predicted_xy[:, 2])
			plotter.predicted_xy_con[index][:set_data](agent.predicted_xy[:, 1], 
											   		   agent.predicted_xy[:, 2])
			plotter.predicted_true_xy[index][:set_data](agent.predicted_true_xy[:, 1], 
										   		   agent.predicted_true_xy[:, 2])
			plotter.predicted_true_xy_con[index][:set_data](agent.predicted_true_xy[:, 1], 
											   		   agent.predicted_true_xy[:, 2])
			plotter.trajectory[index][:set_data](agent.states_xy[1 : agent.current_iteration, 1], 
			     						 		 agent.states_xy[1 : agent.current_iteration, 2])
			plotter.center_of_mass[index][:set_data](center_of_mass[1], center_of_mass[2])
		end

		if plotter.initialization
			for i = 1 : size(agent.selected_states_xy)[1]
				plotter.deleted_states[index, i] =  ax_plot(center_of_mass[1], 
															center_of_mass[2], 
										    			   	"ko")[1]
			end
		else
			for i = 1 : size(agent.selected_states_xy)[1]
				if agent.weights_states[i] > 1
					plotter.deleted_states[index, i][:set_data](agent.selected_states_xy[i, 1], 
											    			   	agent.selected_states_xy[i, 2])
				else
					plotter.deleted_states[index, i][:set_data](center_of_mass[1], 
															 	center_of_mass[2])
				end
			end
		end
	end

	if agent.index == 2
		if plotter.initialization
			plotter.predicted_xy[index] = ax_plot(agent.predicted_xy[:, 1], 
										   		  agent.predicted_xy[:, 2], 
										   		  agent_dot)[1]
			plotter.predicted_xy_con[index] = ax_plot(agent.predicted_xy[:, 1], 
											   		  agent.predicted_xy[:, 2], 
											   		  agent_line)[1]
		else
			plotter.predicted_xy[index][:set_data](agent.predicted_xy[:, 1], 
										   		   agent.predicted_xy[:, 2])
			plotter.predicted_xy_con[index][:set_data](agent.predicted_xy[:, 1], 
											   		   agent.predicted_xy[:, 2])
		end
	end

	if plotter.initialization
		plotter.rect[index] = patch.Rectangle([(- l_rear - wheel_length / 2), 
							   (- width / 2 - wheel_width / 2)], 
							   agent_length + wheel_length, width + wheel_width, 
							   color=agent.color, alpha=0.5, ec="black")

		plotter.front_wheel_left[index] = patch.Rectangle([(- wheel_length / 2), 
							   (- wheel_width / 2)], 
							   wheel_length, wheel_width, 
							   color="gray", alpha=1.0, ec="black")
		plotter.front_wheel_right[index] = patch.Rectangle([(- wheel_length / 2), 
							   (- wheel_width / 2)], 
							   wheel_length, wheel_width, 
							   color="gray", alpha=1.0, ec="black")
		plotter.rear_wheel_left[index] = patch.Rectangle([(- wheel_length / 2), 
							   (- wheel_width / 2)], 
							   wheel_length, wheel_width, 
							   color="gray", alpha=1.0, ec="black")
		plotter.rear_wheel_right[index] = patch.Rectangle([(- wheel_length / 2), 
							   (- wheel_width / 2)], 
							   wheel_length, wheel_width, 
							   color="gray", alpha=1.0, ec="black")

		plotter.ax[:add_artist](plotter.rect[index])
		plotter.ax[:add_artist](plotter.front_wheel_left[index])
		plotter.ax[:add_artist](plotter.front_wheel_right[index])
		plotter.ax[:add_artist](plotter.rear_wheel_left[index])
		plotter.ax[:add_artist](plotter.rear_wheel_right[index])
	else
		plotter.rect[index][:set_transform](set_transformation(plotter.ax, psi, 
											center_of_mass[1], center_of_mass[2]))
		plotter.front_wheel_left[index][:set_transform](set_transformation(plotter.ax, 
														psi_and_steering, 
														center_front_wheel[1, 1], 
														center_front_wheel[1, 2]))
		plotter.front_wheel_right[index][:set_transform](set_transformation(plotter.ax, 
														 psi_and_steering,
														 center_front_wheel[2, 1],
														 center_front_wheel[2, 2]))
		plotter.rear_wheel_left[index][:set_transform](set_transformation(plotter.ax, psi, 
														   center_rear_wheel[1, 1], 
														   center_rear_wheel[1, 2]))
		plotter.rear_wheel_right[index][:set_transform](set_transformation(plotter.ax, psi, 
															center_rear_wheel[2, 1], 
															center_rear_wheel[2, 2]))
	end

	axis("equal")
end

function set_transformation(ax, radians::Float64, trans_x::Float64, 
							trans_y::Float64)
	# try to rotate rectangle using matplotlib's transformations
	t1 = mpl.transforms[:Affine2D]()
	t1[:rotate](radians)
	t1[:translate](trans_x, trans_y)

	# apparently one also has to transform between data coordinate 
	# system and display coordinate system
	t2 = ax[:transData]
	t3 = t1[:__add__](t2)

	return t3
end

function plot_race_info!(plotter::Plotter, agents::Array{Agent},
						 elapsed_time::Float64, num_laps::Int64)
	# elapsed_time = simulator.elapsed_time
	time_string = "Time : $(round(elapsed_time, 2)) s"	

	if length(agents) == 2
	    if agents[1].current_lap > agents[2].current_lap
			leading_index = 1
			following_index = 2
		elseif agents[1].current_lap == agents[2].current_lap
			if agents[1].states_s[agents[1].current_iteration, 1] >
			   agents[2].states_s[agents[2].current_iteration, 1]
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

		leading_agent = ucfirst(agents[leading_index].color)
		leading_lap = agents[leading_index].current_lap
		following_agent = ucfirst(agents[following_index].color)
		following_lap = agents[following_index].current_lap

		race_string = @sprintf "%-10s %-10s\n%-10s %-10s\n%-10s %-10s" "Race" "Lap" "1. $(leading_agent)" "$(leading_lap)/$(num_laps)" "2. $(following_agent)" "$(following_lap)/$(num_laps)"
	else
		leading_agent = ucfirst(agents[1].color)
		leading_lap = agents[1].current_lap
		race_string = @sprintf "%-10s %-10s\n%-10s %-10s" "Race" "Lap" "1. $(leading_agent)" "$(leading_lap)/$(num_laps)"
	end
	# race_title = "Race    Lap\n1. $(leading_agent)   $(leading_lap)/$(simulator.num_laps)\n2. $(following_agent)   $(following_lap)/$(simulator.num_laps)"

	if plotter.initialization
		# plotter.ax[:annotate](time_string, xy=[0.5; 0.9], 
		plotter.time_s = plotter.ax[:annotate]("", xy=[0.5; 0.7], 
										       xycoords= "axes fraction", fontsize=20, 
										       verticalalignment="top")
		# plotter.ax[:annotate](race_string, xy=[0.8; 0.9], xycoords="axes fraction", 
		plotter.race_s = plotter.ax[:annotate]("", xy=[0.5; 0.5], xycoords="axes fraction", 
					  						   fontsize=20, verticalalignment="top")
	else 	
		plotter.time_s[:set_text](time_string)
		plotter.race_s[:set_text](race_string)
	end
end

function replay(agent::Agent, track::Track, total_num_laps::Int64)
	@assert CLEAN_PLOT == true

	# total_num_laps = simulator.num_laps + simulator.num_loaded_laps

	for k = 1 : total_num_laps
		if sumabs(agent.trajectories_s[k, :, :], (2, 3))[1] == 0.
			# println("No data for lap $(lap) available")
			continue
		end
		trajectory = agent.trajectories_xy[k, :, :]	
		needed_iters = findmin(sumabs(agent.trajectories_s[k, :, :], 
									  (1, 3)))[2] - 1 - NUM_CONSIDERED_STATES
		# agent = simulator.agents[1]
		agent.states_xy = squeeze(trajectory, 1)

		for j = 1 : needed_iters - 1
			plt[:clf]()
			plot_track(track)
			plot_finish_line(track)
			# plot_race_info(simulator)

			#=
			for i = 1 : simulator.num_agents
				agent = simulator.agents[i]
				plot_agent(agent, j)
			end
			=#

			plot_agent(agent, j)

			fig = plt[:gcf]()
			fig[:canvas][:set_window_title]("Racing Replay")

			plt[:show]()
			plt[:pause](0.000001)
		end
	end
end