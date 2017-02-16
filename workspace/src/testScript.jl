# File that loads all julia files to do a first basic check for syntax errors
# Saves time since ROS doesn't need to be started

# Michael Garstka, 2017/01/20

include("LMPC_node.jl")
include("barc_lib/LMPC/MPC_models.jl")
include("barc_lib/LMPC/coeffConstraintCost.jl")
include("barc_lib/LMPC/functions.jl")
include("barc_lib/LMPC/solveMpcProblem.jl")
include("barc_lib/classes.jl")