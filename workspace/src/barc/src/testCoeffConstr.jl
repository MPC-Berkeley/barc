using RobotOS
@rosimport barc.msg: ECU, pos_info
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using data_service.msg
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


code = "794b"
log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
#log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
#d_rec       = load(log_path_record)
d_lmpc      = load(log_path_LMPC)

buffersize                  = 2000       # size of oldTraj buffers

# Define and initialize variables
# ---------------------------------------------------------------
# General LMPC variables
posInfo                     = PosInfo()
mpcCoeff                    = MpcCoeff()
lapStatus                   = LapStatus(1,1,false,false,0.3)
mpcSol                      = MpcSol()
trackCoeff                  = TrackCoeff()      # info about track (at current position, approximated)
modelParams                 = ModelParams()
mpcParams                   = MpcParams()
mpcParams_pF                = MpcParams()       # for 1st lap (path following)

InitializeParameters(mpcParams,mpcParams_pF,trackCoeff,modelParams,posInfo,oldTraj,mpcCoeff,lapStatus,buffersize)

oldTraj     = d_lmpc["oldTraj"]


posInfo.s = 5.0
lapStatus.currentLap = 4
oldTraj.count[3] = 

function testperf()
    for i=1:100
        coeffConstraintCost(oldTraj,mpcCoeff,posInfo,mpcParams,lapStatus)
    end
end