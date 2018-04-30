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

# Initialization
init_states = INIT_STATES
v_max = V_MAX[1]
color = COLOR[1]

track = Track()
init!(track)

agent = Agent()
init!(agent, 1, track, MAXIMUM_NUM_ITERATIONS, HORIZON, NUM_LAPS, 
	  NUM_CONSIDERED_STATES, NUM_LOADED_LAPS, v_max, color)
set_state_s!(agent, init_states[1, :], track)

data = load("/home/lukas/simulations/path_following.jld")
all_states = data["trajectories_s"]
inputs = data["previous_inputs"]

# states_s = data["s_states"]
# inputs = data["inputs"]

# mapping = [1, 2, 3, 6, 4, 5]
# states_s = states_s[:, mapping]

# Take data from the first lap
num_considered_states = 100
selected_lap = 1
needed_iterations = findmin(sumabs(all_states[selected_lap, :, :], 3))[2] - 1 - num_considered_states
states_s = all_states[selected_lap, num_considered_states + 1 : needed_iterations, :]

num_data_points = size(states_s, 2)

println("Num Points: ", num_data_points)
println(agent.trajectories_s[1, 1 : 10, :])

for i = 1 : num_data_points
	println(i)
	agent.current_input = inputs[selected_lap, i, :]
	agent.states_s[agent.current_iteration, :] = states_s[selected_lap, i, :]

	identify_system!(agent)
end

theta_vx = agent.t_vx_log[1 : agent.t_counter, :]
theta_vy = agent.t_vy_log[1 : agent.t_counter, :]
theta_psi_dot = agent.t_psi_log[1 : agent.t_counter, :]

num_points = size(theta_vx, 1)
println(size(theta_vx))
println(num_points)

filename = "/home/lukas/simulations/theta_2.jld"
jldopen(filename, "w") do file
    JLD.write(file, "vx", agent.t_vx_log[1 : agent.t_counter, :])
    JLD.write(file, "vy", agent.t_vy_log[1 : agent.t_counter, :])
    JLD.write(file, "psi_dot", agent.t_psi_log[1 : agent.t_counter, :])
end

#=

figure("theta_vx")
plot(1 : num_points, theta_vx[:, 1], label="theta_vx_1", color="blue")
plot(1 : num_points, theta_vx[:, 2], label="theta_vx_2", color="green")
plot(1 : num_points, theta_vx[:, 3], label="theta_vx_3", color="red")
xlabel("iterations")
ylabel("theta_vx_i")
title("theta_vx")
legend(loc="upper right",fancybox="true")
grid("on")

figure("theta_vy")
plot(1 : num_points, theta_vy[:, 1], label="theta_vy_1", color="blue")
plot(1 : num_points, theta_vy[:, 2], label="theta_vy_2", color="green")
plot(1 : num_points, theta_vy[:, 3], label="theta_vy_3", color="red")
plot(1 : num_points, theta_vy[:, 4], label="theta_vy_4", color="cyan")
xlabel("iterations")
ylabel("theta_vy_i")
title("theta_vy")
legend(loc="upper right",fancybox="true")
grid("on")

figure("theta_psi_dot")
plot(1 : num_points, theta_psi_dot[:, 1], label="theta_vx_1", color="blue")
plot(1 : num_points, theta_psi_dot[:, 2], label="theta_vx_2", color="green")
plot(1 : num_points, theta_psi_dot[:, 3], label="theta_vx_3", color="red")
xlabel("iterations")
ylabel("theta_psi_dot_i")
title("theta_psi_dot")
legend(loc="lower left",fancybox="true")
grid("on")
=#
