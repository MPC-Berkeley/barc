#!/usr/bin/env julia

using RobotOS
@rosimport barc.msg: ECU, pos_info, Encoder, Ultrasound, Z_KinBkMdl, Logging
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using JuMP
using Ipopt

include("helper/classes.jl")
include("helper/coeffConstraintCost.jl")
include("helper/solveMpcProblem.jl")
include("helper/computeCostLap.jl")
include("simModel.jl")

# Load Variables and create Model:
println("Loading and defining variables...")
include("createModel.jl")

# Initialize model by solving it once
println("Initial solve...")
solve(mdl)
println("Finished initial solve.")

QderivZ     = 0.0*[1 1 1 1]             # cost matrix for derivative cost of states
QderivU     = 0.1*[1 1]                 # cost matrix for derivative cost of inputs
mpcParams.R = 0.0*[1 1]                 # cost matrix for control inputs
mpcParams.Q = [0.0 10.0 1.0 1.0]        # put weights on ey, epsi and v

cost                        = zeros(length(t),6)
posInfo.s_target            = 6

trackCoeff.coeffCurvature   = [0.0,0.0,0.0,0.0,0.0]         # polynomial coefficients for curvature approximation (zeros for straight line)
z_final = zeros(1,4)
u_final = zeros(1,2)

z_est = zeros(1,4)

function SE_callback(msg::pos_info)         # update current position and track data
    # update mpc initial condition 
    z_est                     = [msg.s msg.ey msg.epsi msg.v]
    posInfo.s_start           = msg.s_start
    trackCoeff.coeffCurvature = msg.coeffCurvature
end

function main()
    # initiate node, set up publisher / subscriber topics
    init_node("mpc_traj")
    pub     = Publisher("ecu", ECU, queue_size=10)
    pub2    = Publisher("logging", Logging, queue_size=10)
    s1      = Subscriber("pos_info", pos_info, SE_callback, queue_size=10)
    loop_rate = Rate(10)

    lapStatus.currentLap    = 1         # initialize lap number
    lapStatus.currentIt     = 1         # current iteration in lap
    switchLap               = false     # initialize lap lap trigger
    s_lapTrigger            = 0.3       # next lap is triggered in the interval s_start in [0,s_lapTrigger]
    
    zCurr       = zeros(10000,4)    # contains state information in current Lap (max. 10'000 steps)
    uCurr       = zeros(10000,2)    # contains input information

    while ! is_shutdown()
        lapStatus.currentIt += 1                        # count iteration
        i                   = lapStatus.currentIt       # just to make notation shorter
        zCurr[i,:]          = z_est                     # update state information

        # Find coefficients for cost and constraints
        mpcCoeff    = coeffConstraintCost(oldTraj,lapStatus,mpcCoeff,posInfo,mpcParams)
        # Solve the MPC problem
        mpcSol      = solveMpcProblem(mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr[i-1,:]',uCurr[i-1,:]')
        # Write in current input information
        uCurr[i,:]  = [mpcSol.a_x mpcSol.d_f]
        # ... and publish data
        cmd = ECU(mpcSol.a_x, mpcSol.d_f)
        publish(pub, cmd)

        # ==================================
        # Lap trigger ======================
        # ==================================
        if posInfo.s_start < s_lapTrigger && switchLap      # if we are switching to the next lap...
            # ... then select and save data
            zCurr_export    = cat(1,zCurr[1:i-1,:], [zCurr[i-1,1]+collect(1:buffersize-i+1)*dt*zCurr[i-1,4] ones(buffersize-i+1,1)*zCurr[i-1,2:4]])
            uCurr_export    = cat(1,uCurr[1:i-1,:], zeros(buffersize-i+1,2))
            costLap         = lapStatus.currentIt               # the cost of the current lap is the time it took to reach the finish line
            # Save all data in oldTrajectory:
            if lapStatus.currentLap == 1                        # if it's the first lap
                oldTraj.oldTraj[:,:,1]  = zCurr_export'         # ... just save everything
                oldTraj.oldInput[:,:,1] = uCurr_export'
                oldTraj.oldTraj[:,:,2]  = zCurr_export'
                oldTraj.oldInput[:,:,2] = uCurr_export'
                oldTraj.oldCost = [costLap,costLap]
            else                                                # idea: always copy the new trajectory in the first array!
                if oldTraj.oldCost[1] < oldTraj.oldCost[2]      # if the first old traj is better than the second
                    oldTraj.oldTraj[:,:,2]  = oldTraj.oldTraj[:,:,1]    # ... copy the first in the second
                    oldTraj.oldInput[:,:,2] = oldTraj.oldInput[:,:,1]   # ... same for the input
                    oldTraj.oldCost[2] = oldTraj.oldCost[1]
                end
                oldTraj.oldTraj[:,:,1]  = zCurr_export'                 # ... and write the new traj in the first
                oldTraj.oldInput[:,:,1] = uCurr_export'
                oldTraj.oldCost[1] = costLap
            end

            lapStatus.currentLap += 1     # start next lap
            lapStatus.currentIt   = 1     # reset current iteration
            switchLap = false
            zCurr[1,:] = zCurr[i,:]
            uCurr[1,:] = uCurr[i,:]
        else
            switchLap = true
        end

        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end