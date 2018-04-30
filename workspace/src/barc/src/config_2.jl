#!/usr/bin/env julia

#=
	File name: config_2.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

const LEARNING = true
const SYS_ID = true

const INITIALIZATION_TYPE = "center"

if LEARNING
	const NUM_LAPS = 10
	const NUM_LOADED_LAPS = 5
	const MODE = "learning" 
else
	const NUM_LAPS = 5
	const NUM_LOADED_LAPS = 0
	const MODE = "path_following"
end

const NUM_AGENTS = 1
const HORIZON = 10
const NUM_CONSIDERED_LAPS = 5
const NUM_CONSIDERED_STATES = 2 * HORIZON * NUM_CONSIDERED_LAPS
const NUM_STATES_BUFFER = 60
const SYS_ID_BEFORE = 25
const SYS_ID_AFTER = 25
@assert NUM_STATES_BUFFER > NUM_CONSIDERED_STATES / NUM_CONSIDERED_LAPS
@assert NUM_STATES_BUFFER > SYS_ID_BEFORE
@assert NUM_STATES_BUFFER > SYS_ID_AFTER
const MAXIMUM_NUM_ITERATIONS = 2000
const INIT_STATES = [0.0 0.0 0.0 0.0 0.0 0.0]
const V_MAX = [2.0; 2.5]
const COLOR = ["red"; "blue"]
const TRACK_NAME = "track_3"
# const TRACK_NAME = "oval"
const TRACK_WIDTH = 1.0
const TRACK_DIR = "/home/lukas/barc/workspace/src/barc/tracks"

if INITIALIZATION_TYPE == "center"
	const CURRENT_REFERENCE = [0.0 0.0 0.0 1.0]
elseif INITIALIZATION_TYPE == "inner"
	const CURRENT_REFERENCE = [0.0 -0.35 0.0 1.0]
elseif INITIALIZATION_TYPE == "outer"
	const CURRENT_REFERENCE = [0.0 0.35 0.0 1.0]
else
	error("INITIALIZATION_TYPE not avialable")
end