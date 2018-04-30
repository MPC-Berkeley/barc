#!/usr/bin/env julia

#=
	File name: config.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

const AVAILABLE_MODES = ["racing"; "learning"; "path_following"; "combining"]

if length(ARGS) >= 1
	const MODE = ARGS[1]
else
	# Choose mode, one in: ["racing", "learning", "path_following"; 
	# 					   "combining"]
	const MODE = "racing"
end

if !(MODE in AVAILABLE_MODES)
	error("The mode $(MODE) is not defined.")
end

# Plotting 
const PLOTTING_2 = true
const PLOTTING = false
# Select if plot should be focused or not
const FOCUSED_PLOT = false
const SAVE_PLOTS = true
# TODO: Set the directory for the plots
const CLEAN_PLOT = false

const SAVE_TRAJECTORIES = false

if MODE == "racing"
	const REFERENCES = [0.0 0.0 0.0 0.0]
	const NUM_LAPS = 10
	const LEARNING = true

elseif MODE == "combining"
	const REFERENCES = [0.0 0.0 0.0 0.0]
	const NUM_LAPS = 0
	const LEARNING = true
else
	const AVAILABLE_FOLLOWING_MODES = ["center"; "outer_bounds";
									   "inner_bounds"]
	if length(ARGS) >= 2
		const PATH_FOLLOWING_MODE = ARGS[2]
	else 
		# Select path following mode, one of: ["center"; "outer_bounds"; 
		#									   "inner_bounds"]
		const PATH_FOLLOWING_MODE = "center"
	end

	if !(PATH_FOLLOWING_MODE in AVAILABLE_FOLLOWING_MODES)
		error("The chosen path following mode $(PATH_FOLLOWING_MODE)", 
				  " is not defined.")
	end

	if MODE == "learning"
		const LEARNING = true
		const REFERENCES = [0.0 0.0 0.0 0.0]
		# More iterations needed for outer starting position to converge
		# to optimal solution
		if PATH_FOLLOWING_MODE == "outer_bounds"
			const NUM_LAPS = 10
		else
			const NUM_LAPS = 10
		end

	elseif MODE == "path_following"
		const LEARNING = false

		if PATH_FOLLOWING_MODE == "center"
			const REFERENCES = [0.0 0.0 0.0 0.6; 0.0 0.0 0.0 0.6]
		elseif PATH_FOLLOWING_MODE == "outer_bounds"
			const REFERENCES = [0.0 0.4 0.0 0.6; 0.0 0.4 0.0 0.6]
		elseif PATH_FOLLOWING_MODE == "inner_bounds"
			const REFERENCES = [0.0 -0.4 0.0 0.6; 0.0 -0.4 0.0 0.6]
		end

		#= 
		# Tested multiple path following laps with different e_y and high
		# velocities
		const REFERENCES = [0.0 0.8 0.0 2.3; 
					  		0.0 0.6 0.0 2.3;
					  		0.0 0.4 0.0 2.3; 
					  		0.0 0.2 0.0 2.3;
					  		0.0 0.0 0.0 2.3; 
					  		0.0 -0.2 0.0 2.3;
					  		0.0 -0.4 0.0 2.3; 
					  		0.0 -0.6 0.0 2.3;
					  		0.0 -0.8 0.0 2.3;]
		=#

		if size(REFERENCES)[1] < 2
			error("At least two laps of path following needed to create the ",
				  "convex hull.")
		end

		const NUM_LAPS = size(REFERENCES)[1]
	end
end

if !(MODE == "racing")
	const NUM_AGENTS = 1
	if !(MODE == "combining")
		if PATH_FOLLOWING_MODE == "center"
			const INIT_STATES = [0.0 0.0 0.0 0.4]
		elseif PATH_FOLLOWING_MODE == "outer_bounds"
			const INIT_STATES = [0.0 0.4 0.0 0.4]
		elseif PATH_FOLLOWING_MODE == "inner_bounds"
			const INIT_STATES = [0.0 -0.4 0.0 0.4]
		end
	else
		const INIT_STATES = [0.0 0.0 0.0 0.6]
		# const INIT_STATES = [0.0 1.0 0.0 0.4]
		# const INIT_STATES = [0.0 -1.0 0.0 0.4]
	end
else
	# Choose the number of agents, one of [1; 2]
	const NUM_AGENTS = 2
	const INIT_STATES = [2.0 0.0 0.0 0.4; 0.0 0.0 0.0 0.4]
	# const INIT_STATES = [1.6 0.0 0.0 0.4; 1.0 0.0 0.0 0.4]
end

if NUM_AGENTS > 2
	error("Currently only one or two agents can race.")
end

const V_MAX = [2.0; 2.5]
# const V_MAX = [2.5]
if V_MAX[1] < 2.5
	speed_string = "_slow"
else 
	speed_string = ""
end

const COLOR = ["red"; "blue"]

if size(INIT_STATES)[1] < NUM_AGENTS
	error("Not able to initialize the starting state for every agent.")
end
if size(V_MAX)[1] < NUM_AGENTS
	error("Not able to initialize the maximum velocity for every agent.")
end
if size(COLOR)[1] < NUM_AGENTS
	error("Not able to initialize the color for every agent.")
end

# Choose the track, one of ["oval"; "track_3"]
const AVAILABLE_TRACKS = ["oval"; "track_3"]
const TRACK_NAME = "track_3"
const TRACK_WIDTH = 1.0

if !(TRACK_NAME in AVAILABLE_TRACKS)
	error("Track $(TRACK_NAME) is not available")
end

const HORIZON = 10
const NUM_CONSIDERED_LAPS = 5
const NUM_CONSIDERED_STATES = NUM_CONSIDERED_LAPS * 2 * HORIZON  # per lap

# Set the filenames
if MODE == "learning"
	#=
	const LOAD_FILENAMES = ascii("../trajectories/" * TRACK_NAME * 
						   		 "/path_following_" * PATH_FOLLOWING_MODE * 
						   		 ".jld")
	const SAVE_FILENAME = ascii("../trajectories/" * TRACK_NAME * 
						 	    "/learning_mpc_" * PATH_FOLLOWING_MODE * 
						 	    ".jld")
	=#
	const LOAD_FILENAMES = [ascii("../trajectories/" * TRACK_NAME * 
						   		 "/path_following_" * PATH_FOLLOWING_MODE * 
						   		 speed_string * ".jld")]
	const SAVE_FILENAME = ascii("../trajectories/" * TRACK_NAME * 
						 	    "/learning_mpc_" * PATH_FOLLOWING_MODE * 
						 	    speed_string * ".jld")
elseif MODE == "racing"
	# const LOAD_FILENAMES = ascii("../trajectories/" * TRACK_NAME * 
	#					   		   "/learning_mpc_center.jld")
	# const LOAD_FILENAMES = ascii("../trajectories/" * TRACK_NAME * 
	#					   		   "/learning_mpc_outer_bounds.jld")
	# const LOAD_FILENAMES = ascii("../trajectories/" * TRACK_NAME * 
	#					   		   "/learning_mpc_inner_bounds.jld")
	const LOAD_FILENAMES = [ascii("../trajectories/" * TRACK_NAME * 
					   	   		 "/learning_mpc_combined_initialization_slow.jld");
							ascii("../trajectories/" * TRACK_NAME * 
					   	   		 "/learning_mpc_combined_initialization.jld")]

	if SAVE_TRAJECTORIES
		const SAVE_FILENAME = ascii("../trajectories/" * TRACK_NAME * 
							  		"/racing_mpc.jld")
	end
elseif MODE == "combining"
	# Combine previously recorded data
	#=
	const LOAD_FILENAMES = [ascii("../trajectories/" * TRACK_NAME * 
						    	  "/learning_mpc_center.jld"); 
						    ascii("../trajectories/" * TRACK_NAME * 
						   		  "/learning_mpc_outer_bounds.jld"); 
						    ascii("../trajectories/" * TRACK_NAME * 
						    	  "/learning_mpc_inner_bounds.jld")]
	=#

	#=
	const LOAD_FILENAMES = [ascii("../trajectories/" * TRACK_NAME * 
						    	  "/learning_mpc_center.jld"); 
						    ascii("../trajectories/" * TRACK_NAME * 
						   		  "/learning_mpc_outer_bounds.jld")]
	const SAVE_FILENAME = ascii("../trajectories/" * TRACK_NAME * 
					   	   		"/learning_mpc_combined_initialization.jld")
	=#

	const LOAD_FILENAMES = [ascii("../trajectories/" * TRACK_NAME * 
						    	  "/learning_mpc_center" * speed_string * ".jld"); 
						    ascii("../trajectories/" * TRACK_NAME * 
						   		  "/learning_mpc_outer_bounds" * speed_string * ".jld");
						    ascii("../trajectories/" * TRACK_NAME * 
						   		  "/learning_mpc_inner_bounds" * speed_string * ".jld")]
	const SAVE_FILENAME = ascii("../trajectories/" * TRACK_NAME * 
					   	   		"/learning_mpc_combined_initialization" * speed_string * ".jld")
else  # MODE == "path following"
	const LOAD_FILENAMES = nothing
	const SAVE_FILENAME = ascii("../trajectories/" * TRACK_NAME * 
								"/path_following_" * PATH_FOLLOWING_MODE * 
								".jld")
	#=
	const SAVE_FILENAME = ascii("../trajectories/" * TRACK_NAME * 
								"/path_following_" * PATH_FOLLOWING_MODE * 
								"_fast" * ".jld")
	=#
end