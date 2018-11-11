#!/usr/bin/env julia

#=
	File name: config.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

import YAML

config_file = ENV["HOME_DIR"] * "/barc/workspace/src/barc/src/config.yaml"
config = YAML.load(open(config_file))

const MODE = config["mode"]

if MODE == "racing"
	# const NUM_AGENTS = 1
	const NUM_AGENTS = 2
else 
	const NUM_AGENTS = 1
end

# const SYS_ID = true

const INITIALIZATION_TYPE = config["initialization"]

if MODE == "path_following"
	const NUM_PF_LAPS = config["num_pf_laps"]
	const LEARNING = false
	const NUM_LAPS = NUM_PF_LAPS
	const NUM_LOADED_LAPS = 0
elseif MODE == "learning"
	const NUM_PF_LAPS = 1
	const LEARNING = true
	const NUM_LAPS = config["num_laps"]
	const NUM_LOADED_LAPS = config["num_pf_laps"]
elseif MODE == "racing"
	const NUM_PF_LAPS = 1
	const LEARNING = true
	const NUM_LAPS = config["num_laps"]
	const NUM_LOADED_LAPS = config["num_pf_laps"] + 3 * config["num_laps"]
end

const HORIZON = config["horizon"]
const NUM_CONSIDERED_LAPS = config["num_considered_laps"]
const NUM_HORIZONS = config["num_horizons"]
# const SELECTION_SHIFT = 0
const SELECTION_SHIFT = Int64(HORIZON / 2) # round(Int64, 1 * HORIZON) 
const NUM_CONSIDERED_STATES = round(Int64, NUM_HORIZONS * HORIZON) * NUM_CONSIDERED_LAPS

const NUM_STATES_BUFFER = config["num_states_buffer"]
const SYS_ID_BEFORE = config["sys_id_before"]
const SYS_ID_AFTER = config["sys_id_after"]
@assert NUM_STATES_BUFFER > NUM_CONSIDERED_STATES / NUM_CONSIDERED_LAPS
@assert NUM_STATES_BUFFER > SYS_ID_BEFORE
@assert NUM_STATES_BUFFER > SYS_ID_AFTER

const MAXIMUM_NUM_ITERATIONS = config["maximum_num_iterations"]
const INIT_STATES = convert(Array{Float64,2}, reshape(config["init_states"], (1, 6)))
const V_MAX = config["v_max"]
const COLOR = config["color"]
const TRACK_NAME = config["track_name"]
const TRACK_WIDTH = config["track_width"]
const POLYNOMIAL_CURVATURE = false

const V_INIT = config["v_init"]
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

const FOCUSED_PLOT = config["focused_plot"]
const SAVE_PLOTS = config["save_plots"]
