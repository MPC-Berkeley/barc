#!/usr/bin/env julia

#=
	File name: config_2.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

const MODE = "path_following"
# const MODE = "learning"
# const MODE = "racing"

if MODE == "racing"
	# const NUM_AGENTS = 1
	const NUM_AGENTS = 2
else 
	const NUM_AGENTS = 1
end

# const SYS_ID = true

const INITIALIZATION_TYPE = "center"

if MODE == "path_following"
	const NUM_PF_LAPS = 5
	const LEARNING = false
	const NUM_LAPS = NUM_PF_LAPS
	const NUM_LOADED_LAPS = 0
elseif MODE == "learning"
	const NUM_PF_LAPS = 1
	const LEARNING = true
	const NUM_LAPS = 30
	const NUM_LOADED_LAPS = 5
elseif MODE == "racing"
	const NUM_PF_LAPS = 1
	const LEARNING = true
	const NUM_LAPS = 30
	# const NUM_LOADED_LAPS = 5 + 10
	const NUM_LOADED_LAPS = 5 + 30 + 30 + 30

	# const NUM_LOADED_LAPS = 5
end

const HORIZON = 10
const NUM_CONSIDERED_LAPS = 4
const NUM_HORIZONS = 2.0
# const SELECTION_SHIFT = 0
const SELECTION_SHIFT = Int64(HORIZON / 2) # round(Int64, 1 * HORIZON) 
# const NUM_CONSIDERED_STATES = 2 * HORIZON * NUM_CONSIDERED_LAPS
const NUM_CONSIDERED_STATES = round(Int64, NUM_HORIZONS * HORIZON) * NUM_CONSIDERED_LAPS
const NUM_STATES_BUFFER = 30
const SYS_ID_BEFORE = 15
const SYS_ID_AFTER = 15
@assert NUM_STATES_BUFFER > NUM_CONSIDERED_STATES / NUM_CONSIDERED_LAPS
@assert NUM_STATES_BUFFER > SYS_ID_BEFORE
@assert NUM_STATES_BUFFER > SYS_ID_AFTER
const MAXIMUM_NUM_ITERATIONS = 2000
const INIT_STATES = [0.0 0.0 0.0 0.0 0.0 0.0]
const V_MAX = [2.0; 2.5]
const COLOR = ["red"; "blue"]
# const TRACK_NAME = "track_3"
# const TRACK_NAME = "oval"
const TRACK_NAME = "l_shape"
const TRACK_WIDTH = 1.2
const POLYNOMIAL_CURVATURE = false

const V_INIT = 1.2
const EY_CENTER = 0.0
const EY_OFFSET = 0.75 * TRACK_WIDTH / 2
const EY_INNER = - EY_OFFSET
const EY_OUTER = EY_OFFSET

if INITIALIZATION_TYPE == "center"
	const CURRENT_REFERENCE = [0.0 EY_CENTER 0.0 V_INIT]
elseif INITIALIZATION_TYPE == "inner"
	const CURRENT_REFERENCE = [0.0 EY_INNER 0.0 V_INIT]
elseif INITIALIZATION_TYPE == "outer"
	const CURRENT_REFERENCE = [0.0 EY_OUTER 0.0 V_INIT]
else
	error("INITIALIZATION_TYPE $(INITIALIZATION_TYPE) not available.")
end

const FOCUSED_PLOT = false
const SAVE_PLOTS = false