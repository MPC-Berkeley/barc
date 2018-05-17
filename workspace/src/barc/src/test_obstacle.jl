#!/usr/bin/env julia

using RobotOS
@rosimport barc.msg: ECU, pos_info, prediction
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using geometry_msgs.msg
using JuMP
using Ipopt
using JLD

# log msg
include("barc_lib/classes.jl")
include("barc_lib/LMPC/functions.jl")
include("barc_lib/LMPC/MPC_models.jl")
include("barc_lib/LMPC/coeffConstraintCost.jl")
include("barc_lib/LMPC/solveMpcProblem.jl")
include("barc_lib/simModel.jl")
include("barc_lib/obstaclePosition.jl")

# horizon
const OBSTACLE_N = 12

function obstacle()
	obstacle_predictions = prediction()

	init_node("obstacle")
	loop_rate = Rate(10.0)

	obstacle = Publisher("obs_prediction", prediction, 
						 queue_size=1)::RobotOS.Publisher{barc.msg.prediction}

	while !is_shutdown()
		obstacle_predictions.header.stamp = get_rostime()
		obstacle_predictions.s = 12 * ones(OBSTACLE_N + 1)
		obstacle_predictions.ey = - 0.35 * ones(OBSTACLE_N + 1)
		obstacle_predictions.epsi = 0.0 * ones(OBSTACLE_N + 1)
		obstacle_predictions.v = 0.0 * ones(OBSTACLE_N + 1)

		publish(obstacle, obstacle_predictions)	

		rossleep(loop_rate)
	end
end

if !isinteractive()
    obstacle()
end