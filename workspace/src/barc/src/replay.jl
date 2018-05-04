#!/usr/bin/env julia

#=
	File name: replay.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#


using PyCall
using PyPlot
using JLD
using QuickHull

@pyimport matplotlib.patches as patch
@pyimport matplotlib.transforms as transforms
@pyimport matplotlib.animation as animation
@pyimport matplotlib as mpl
@pyimport matplotlib.collections as collections

# cv2 = pyimport("cv2")

include("config_2.jl")
include("filesystem_helpers.jl")


function plot_selection(ax, x, y, color)
	# Plot convex hull of the selected_states
	conv_hull_temp = qhull(x, y)
	conv_hull = zeros(size(conv_hull_temp[1])[1] + 1, 2)
	conv_hull[1 : end - 1, 1] = conv_hull_temp[1]
	conv_hull[1 : end - 1, 2] = conv_hull_temp[2]
	conv_hull[end, 1] = conv_hull[1, 1]
	conv_hull[end, 2] = conv_hull[1, 2]

	convex_hull = patch.Polygon(conv_hull, fill=true, color=color, alpha=0.2)
	ax[:add_artist](convex_hull)

	plt_hull_bounds, = ax[:plot](conv_hull[:, 1], conv_hull[:, 2], color * "-", lw=2.0)
	plt_states, = ax[:plot](x, y, color * "o")

	counter = 1
	states_per_lap = round(Int64, size(x, 1) / NUM_CONSIDERED_LAPS)
	plt_laps = []
	for i = 1 : NUM_CONSIDERED_LAPS
		x_lap_i = x[counter : counter + states_per_lap - 1]
		y_lap_i = y[counter : counter + states_per_lap - 1]
		dummy_lap, = ax[:plot](x_lap_i, y_lap_i, color * "-")
		plt_laps = [plt_laps; dummy_lap]
		counter += states_per_lap
	end

	return convex_hull, plt_hull_bounds, plt_states, plt_laps
end


function update_selection(convex_hull, plt_hull_bounds, plt_states, plt_laps, x, y)
	# Plot convex hull of the selected_states
	conv_hull_temp = qhull(x, y)
	conv_hull = zeros(size(conv_hull_temp[1])[1] + 1, 2)
	conv_hull[1 : end - 1, 1] = conv_hull_temp[1]
	conv_hull[1 : end - 1, 2] = conv_hull_temp[2]
	conv_hull[end, 1] = conv_hull[1, 1]
	conv_hull[end, 2] = conv_hull[1, 2]

	convex_hull[:set_xy](conv_hull)
	plt_hull_bounds[:set_data](conv_hull[:, 1], conv_hull[:, 2])
	plt_states[:set_data](x, y)

	counter = 1
	states_per_lap = round(Int64, size(x, 1) / NUM_CONSIDERED_LAPS)
	for i = 1 : NUM_CONSIDERED_LAPS
		x_lap_i = x[counter : counter + states_per_lap - 1]
		y_lap_i = y[counter : counter + states_per_lap - 1]
		plt_laps[i][:set_data](x_lap_i, y_lap_i)
		counter += states_per_lap
	end
end


function plot_prediction(ax, x, y, color)
	prediction_dots, = ax[:plot](x[1 : end - 1], y[1 : end - 1], color * "o")
	prediction_star, = ax[:plot](x[end], y[end], color * "*", markersize=10)
	prediction_line, = ax[:plot](x, y, color * "-")

	return prediction_dots, prediction_star, prediction_line
end


function update_prediction(dots, star, line, x, y)
	dots[:set_data](x[1 : end - 1], y[1 : end - 1])
	star[:set_data](x[end], y[end])
	line[:set_data](x, y)
end


function update_limits(ax, x, y, x_pred, y_pred)
	ax[:set_xlim]([min(findmin(x)[1], findmin(x_pred)[1]) - 0.1,
			   max(findmax(x)[1], findmax(x_pred)[1]) + 0.1])
	# ax[:set_ylim]([min(findmin(y)[1], findmin(y_pred)[1]),
	# 		   max(findmax(y)[1], findmax(y_pred)[1])])
end

function update_limits(ax, x_pred, y_pred)
	ax[:set_xlim]([findmin(x_pred)[1] - 0.1,
			       findmax(x_pred)[1] + 0.1])
	# ax[:set_ylim]([min(findmin(y)[1], findmin(y_pred)[1]),
	# 		   max(findmax(y)[1], findmax(y_pred)[1])])
end


function set_limits(ax, trajectories_s, predictions, index, indeces)
	min_val = min(findmin(trajectories_s[:, :, index])[1], 
			 findmin(predictions[indeces, :, index])[1])
	max_val = max(findmax(trajectories_s[:, :, index])[1], 
			 findmax(predictions[indeces, :, index])[1])
	ax[:set_ylim]([min_val - 0.1, max_val + 0.1])
end


function replay_prediction(file)
	plt[:ion]()

	# Create the name of the window
	figure_string = create_window_name(file)

	index = get_index(file)
	if index == 1
		color = "b"
		extra_color = "c"
	elseif index == 2
		color = "r"
		extra_color = "m"
	else
		error("Only two colors defined.")
	end

	data = load(file)

	# Create figure and subplots
	fig = figure("Prediction " * figure_string)
	ax_s_ey = fig[:add_subplot](2, 3, 1)
	xlabel("s [m]")
	ylabel("e_y [m]")

	ax_s_epsi = fig[:add_subplot](2, 3, 2)
	xlabel("s [m]")
	ylabel("e_psi [rad]")

	ax_s_psidot = fig[:add_subplot](2, 3, 3)
	xlabel("s [m]")
	ylabel("psi_dot [rad / s]")

	ax_s_vx = fig[:add_subplot](2, 3, 4)
	xlabel("s [m]")
	ylabel("v_x [m / s]")

	ax_s_vy = fig[:add_subplot](2, 3, 5)
	xlabel("s [m]")
	ylabel("v_y [m / s]")

	ax = [ax_s_ey, ax_s_epsi, ax_s_psidot, ax_s_vx, ax_s_vy]

	trajectories_s = data["trajectories_s"]
	all_selected_states = data["all_selected_states"]
	all_predictions = data["all_predictions"]

	# Plot track bounds
	ax_s_ey[:plot]([- 5; 20], [TRACK_WIDTH / 2; TRACK_WIDTH / 2], "k-", lw=2.0)
	ax_s_ey[:plot]([- 5; 20], [- TRACK_WIDTH / 2; - TRACK_WIDTH / 2], "k-", lw=2.0)
	ax_s_ey[:plot]([- 5; 20], [0; 0], "k--")

	# Plot finish lines
	max_s = ceil(findmax(all_predictions[:, 1, 1])[1] * 10) / 10
	ax_s_ey[:plot]([0; 0], [- TRACK_WIDTH / 2; TRACK_WIDTH / 2], "k--", lw=5.0)
	ax_s_ey[:plot]([max_s; max_s], [- TRACK_WIDTH / 2; TRACK_WIDTH / 2], "k--", lw=5.0)

	# Plot reference
	ey_init = 0.0
	epsi_init = 0.0
	v_init = V_INIT
	if contains(file, "center")
		ey_init = EY_CENTER
	elseif contains(file, "inner")
		ey_init = EY_INNER
	elseif contains(file, "outer")
		ey_init = EY_OUTER
	else
		error("No initalization value for ey_init found.")
	end

	ey_ref, = ax_s_ey[:plot]([- 5; 20], [ey_init; ey_init], color * "-", lw=2.0)
	epsi_ref, = ax_s_epsi[:plot]([- 5; 20], [epsi_init; epsi_init], color * "-", lw=2.0)
	vx_ref, = ax_s_vx[:plot]([- 5; 20], [v_init; v_init], color * "-", lw=2.0)

	# Determine the start and end index
	start_index = 0
	sum_over_predictions = sumabs(all_predictions[:, :, [4, 6]], (2, 3))
	for i = 1 : 6000
		if sum_over_predictions[i] > 1e-5
			start_index = i
			break
		end
	end

	end_index = 0
	for i = 6000 : - 1 : 1
		if sum_over_predictions[i] > 1e-5
			end_index = i
			break
		end
	end

	for i = 1 : 5
		set_limits(ax[i], trajectories_s, all_predictions, i + 1, start_index : end_index)
	end

	iteration = 1
	predicted_s = squeeze(all_predictions[iteration, :, :], 1)

	dots = []
	stars = []
	lines = []

	for i = 1 : 5
		dot_plt, star, line = plot_prediction(ax[i], predicted_s[:, 1],
										      predicted_s[:, i + 1], color)
		dots = [dots; dot_plt]
		stars = [stars; star]
		lines = [lines; line]
	end

	for iteration = 2 : start_index
		predicted_s = squeeze(all_predictions[iteration, :, :], 1)

		for i = 1 : 5
			update_prediction(dots[i], stars[i], lines[i], predicted_s[:, 1],
							  predicted_s[:, i + 1])
			update_limits(ax[i], predicted_s[:, 1], predicted_s[:, i + 1])
		end

		plt[:pause](0.05)
	end

	test_selection = all_selected_states[start_index]
	selected_states = zeros(1, 6)

	for i = 1 : NUM_CONSIDERED_LAPS
		dummy = squeeze(trajectories_s[test_selection[i][1], test_selection[i][2], :], 1)
		selected_states = [selected_states; dummy]
	end

	# Make the references invisible after the path following
	ey_ref[:set_alpha](0.0)
	epsi_ref[:set_alpha](0.0)
	vx_ref[:set_alpha](0.0)

	selected_states = selected_states[2 : end, :]

	conv_hulls = []
	hull_bounds = []
	plt_states = []
	plt_laps = []

	for i = 1 : 5
		conv_hull, hull_bound, plt_state, plt_lap = plot_selection(ax[i], 
																   selected_states[:, 1], 
																   selected_states[:, i + 1], 
																   extra_color)
		conv_hulls = [conv_hulls; conv_hull]
		hull_bounds = [hull_bounds; hull_bound]
		plt_states = [plt_states; plt_state]
		plt_laps = [plt_laps; plt_lap]
	end

	for iteration = start_index + 1 : end_index
		test_selection = all_selected_states[iteration]
		predicted_s = squeeze(all_predictions[iteration, :, :], 1)

		selected_states = zeros(1, 6)

		for i = 1 : NUM_CONSIDERED_LAPS
			dummy = squeeze(trajectories_s[test_selection[i][1], 
										   test_selection[i][2], :], 1)
			selected_states = [selected_states; dummy]
		end

		selected_states = selected_states[2 : end, :]

		for i = 1 : 5
			update_selection(conv_hulls[i], hull_bounds[i], plt_states[i], 
							 plt_laps[1 + (i - 1) * NUM_CONSIDERED_LAPS : i * NUM_CONSIDERED_LAPS], 
							 selected_states[:, 1], 
							 selected_states[:, i + 1])

			update_prediction(dots[i], stars[i], lines[i], predicted_s[:, 1],
							  predicted_s[:, i + 1])
			update_limits(ax[i], predicted_s[:, 1], predicted_s[:, i + 1])
		end

		plt[:pause](0.05)
	end
end


function replay_data(file)
	# Create the name of the window
	figure_string = create_window_name(file)

	index = get_index(file)
	if index == 1
		color = "b"
		extra_color = "c"
	elseif index == 2
		color = "r"
		extra_color = "m"
	else
		error("Only two colors defined.")
	end

	# Create figure and subplots
	fig = figure("States " * figure_string)

	ax_s = fig[:add_subplot](3, 2, 1)
	xlabel("iteration")
	ylabel("s [m]")
	ax_ey = fig[:add_subplot](3, 2, 2)
	xlabel("iteration")
	ylabel("e_y [m]")
	ax_epsi = fig[:add_subplot](3, 2, 3)
	xlabel("iteration")
	ylabel("e_psi [rad]")
	ax_psidot = fig[:add_subplot](3, 2, 4)
	xlabel("iteration")
	ylabel("psi_dot [rad / s]")
	ax_vx = fig[:add_subplot](3, 2, 5)
	xlabel("iteration")
	ylabel("v_x [m / s]")
	ax_vy = fig[:add_subplot](3, 2, 6)
	xlabel("iteration")
	ylabel("v_y [m / s]")

	data = load(file)
	trajectories_s = data["trajectories_s"]
	trajectories_s = trajectories_s[:, NUM_STATES_BUFFER + 1 : end, :]
	trajectories_s[1, :, :] = data["trajectories_s"][1, 1 : end - NUM_STATES_BUFFER, :]

	start_indeces = zeros(Int64, size(trajectories_s, 1))
	end_indeces = zeros(Int64, size(trajectories_s, 1))

	# Find maximum num iteration for each lap
	min_indeces = findmin(sumabs(trajectories_s[:, :, :], (3)), 2)[2]

	for i = 1 : size(trajectories_s, 1)
		index = ind2sub(trajectories_s[:, :, 1], min_indeces[i])[2]
		if i == size(trajectories_s, 1)
			end_indeces[i] = index - 1
		else
			end_indeces[i] = index - 1 - NUM_STATES_BUFFER
		end
		# println(trajectories_s[i, end_indeces[i], 1], " < ", trajectories_s[1, end_indeces[1], 1] - 1)
		if trajectories_s[i, end_indeces[i], 1] < trajectories_s[1, end_indeces[1], 1] - 1
			end_indeces[i] += NUM_STATES_BUFFER
		end
		# println(trajectories_s[i, end_indeces[i], 1])
	end
	# println(end_indeces)

	num_iterations = 0
	for i = 1 : size(trajectories_s, 1)
		num_iterations += end_indeces[i]
	end

	states_s = zeros(num_iterations, 6)
	counter = 1
	for i = 1 : size(trajectories_s, 1)
		states_s[counter : counter + end_indeces[i] - 1, :] = squeeze(trajectories_s[i, 1 : end_indeces[i], :], 1)
		counter += end_indeces[i]
	end

	println(size(states_s))

	ax_s[:plot](1 : num_iterations, states_s[:, 1], "r-")
	ax_ey[:plot](1 : num_iterations, states_s[:, 2], "r-")
	ax_epsi[:plot](1 : num_iterations, states_s[:, 3], "r-")
	ax_psidot[:plot](1 : num_iterations, states_s[:, 4], "r-")
	ax_vx[:plot](1 : num_iterations, states_s[:, 5], "r-")
	ax_vy[:plot](1 : num_iterations, states_s[:, 6], "r-")


	plt[:show]()

	exit()

end


function replay()
	if length(ARGS) < 1 
		error("No argument given. To start the replay specify the argument like this:

	    julia replay.jl <argument> (optional: <file>)

	    For <argument>, try one of: prediction, recording or data.
	    For <file>, specify the file by giving the date of the recording: 
	    e. g. 0503 (replays most recent file from that folder) or 
	    0503115854 (replays the file with this timestamp).
	    ")
	elseif length(ARGS) > 2
		error("Too many arguments given. To start the replay specify the argument like this:

		julia replay.jl <argument> (optional: <file>)

		For <argument>, try one of: prediction, recording or data.
		For <file>, specify the file by giving the date of the recording: 
		e. g. 0503 (replays most recent file from that folder) or 
   	    0503115854 (replays the file with this timestamp).
   	    ")
	end

	if length(ARGS) >= 2
		file = find_file(ARGS[2])
	else 
		# Load most recently recorded file
		file = get_most_recent_file()
	end

	if ARGS[1] == "prediction"
		replay_prediction(file)
	elseif ARGS[1] == "recording"
		replay_recording(file)
	elseif ARGS[1] == "data"
		replay_data(file)
	else 
		error("Unknown argument $(ARGS[1]). Try one of: prediction, recording 
			   or data.")
	end
end

# run the replay function
replay()