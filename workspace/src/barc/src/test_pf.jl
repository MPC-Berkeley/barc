#!/usr/bin/env julia
using RobotOS
@rosimport barc.msg: ECU, pos_info, Vel_est, prediction, xy_prediction, theta, selected_states
rostypegen()

using barc.msg

using JLD
using PyPlot

include("config_2.jl")
include("track.jl")
include("transformations.jl")
include("agent.jl")
include("optimizer.jl")

# Load recorded data
data = load("/home/lukas/simulations/states_infeasible.jld")
recorded_s = data["trajectories_s"]
recorded_inputs = data["previous_inputs"]

# Initialization
init_states = [0.0 0.0 0.0 0.0 1.0 0.0] # s, e_y, e_psi, psi_dot, v_x, v_y
v_max = V_MAX[1]
color = COLOR[1]
# 5-States: s, e_y, e_psi, v_x, v_y
reference = [0.0 0.0 0.0 1.0 0.0] 

track = Track()
init!(track)

agent = Agent()
init!(agent, 1, track, MAXIMUM_NUM_ITERATIONS, HORIZON, NUM_LAPS, 
	  NUM_CONSIDERED_STATES, NUM_LOADED_LAPS, v_max, color)
set_state_s!(agent, init_states[1, :], track)

optimizer = Optimizer()
init!(optimizer, agent, HORIZON)

init_path_following_regression!(optimizer, track)

warm_start_states = zeros(size(agent.predicted_s, 1), 5)
warm_start_states[:, 4] = reference[4]
warm_start_states[:, 1] = cumsum(agent.dt * warm_start_states[:, 4])
println("warm start states: ", warm_start_states)

# Warm start prediction
setvalue(optimizer.states_s, warm_start_states)


# agent.theta_vx = [0.05; 0.0; 0.1]
# agent.theta_vy = [0.0; 0.0; 0.0; 1.0]
# agent.theta_psi_dot = [0.0; 0.0; 1.0]

# println("THETA VX: ", agent.theta_vx)
# println("THETA VY: ", agent.theta_vy)
# println("THETA PSI DOT: ", agent.theta_psi_dot)

agent.states_s[1, :] = recorded_s[1, :]
agent.inputs[1, :] = recorded_inputs[1, :]

# for i = 1 : 100
for i = 1 : 30 # size(recorded_s, 1)
	println("Iteration: ", i)

	# Apply system identification
	identify_system!(agent)

	# Solve the optimization problem
	solve_path_following_regression!(optimizer, track, reference)

	println("Predicted s: ", optimizer.solution_states_s)
	println("Predicted input: ", optimizer.solution_inputs)

	# Set the new states and inputs 
	agent.current_iteration += 1
	# agent.states_s[agent.current_iteration, [1; 2; 3; 5; 6]] = optimizer.solution_states_s[2, :]
	# agent.inputs[agent.current_iteration - 1, :] = optimizer.solution_inputs[1, :]
	agent.states_s[agent.current_iteration, :] = recorded_s[agent.current_iteration, :]
	agent.inputs[agent.current_iteration - 1, :] = recorded_inputs[agent.current_iteration - 1, :]
	
end