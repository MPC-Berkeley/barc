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

# Load Variables and create Model:
println("Loading and defining variables...")
include("helper/createModel.jl")

# Initialize model by solving it once
println("Initial solve...")
solve(mdl)
println("Finished initial solve.")


z_est = zeros(1,4)


posInfo                     = PosInfo()
posInfo.s_start             = 0
posInfo.s_target            = 10.2812

function SE_callback(msg::pos_info)         # update current position and track data
    # update mpc initial condition
    global trackCoeff, z_est, posInfo
    z_est                     = [msg.s msg.ey msg.epsi msg.v]
    posInfo.s                 = msg.s
    posInfo.s_start           = msg.s_start
    trackCoeff.coeffCurvature = msg.coeffCurvature
    #println("Received pos info: $msg")
    #println("coeffCurvature = $(trackCoeff.coeffCurvature)")
end

function main()
    println("now starting the node")
    # initiate node, set up publisher / subscriber topics
    init_node("mpc_traj")
    pub     = Publisher("ecu", ECU, queue_size=10)
    pub2    = Publisher("logging", Logging, queue_size=10)
    s1      = Subscriber("pos_info", pos_info, SE_callback, queue_size=10)
    loop_rate = Rate(10)

    # Define and initialize variables
    global mpcParams, trackCoeff, modelParams, z_est, posInfo
    oldTraj                     = OldTrajectory()
    mpcCoeff                    = MpcCoeff()
    lapStatus                   = LapStatus(1,1)
    mpcSol                      = MpcSol()

    mpcParams.QderivZ           = 0.0*[1 1 1 1]             # cost matrix for derivative cost of states
    mpcParams.QderivU           = 0.1*[1 1]                 # cost matrix for derivative cost of inputs
    mpcParams.R                 = 0.0*[1 1]                 # cost matrix for control inputs
    mpcParams.Q                 = [0.0 10.0 1.0 1.0]        # put weights on ey, epsi and v

    buffersize                  = 700
    oldTraj.oldTraj             = zeros(4,buffersize,2)
    oldTraj.oldInput            = zeros(2,buffersize,2)

    mpcCoeff.coeffCost          = 0
    mpcCoeff.coeffConst         = 0
    mpcCoeff.order              = 5
    mpcCoeff.pLength            = 4*mpcParams.N        # small values here may lead to numerical problems since the functions are only approximated in a short horizon

    lapStatus.currentLap        = 1         # initialize lap number
    lapStatus.currentIt         = 1         # current iteration in lap



    # Lap parameters
    switchLap               = false     # initialize lap lap trigger
    s_lapTrigger            = 0.3       # next lap is triggered in the interval s_start in [0,s_lapTrigger]
    
    # buffer in current lap
    zCurr       = zeros(10000,4)    # contains state information in current Lap (max. 10'000 steps)
    uCurr       = zeros(10000,2)    # contains input information

    println("Starting loop")
    while ! is_shutdown()

        # global trackCoeff, mpcParams   # do they need to be within the while loop?
        if posInfo.s_start + posInfo.s >= posInfo.s_target      # if we are behind the finish line
            posInfo.s_start = 0                                         # then set s_start to 0
            posInfo.s = posInfo.s_start + posInfo.s - posInfo.s_target  # and calculate new current s
        end

        lapStatus.currentIt += 1                        # count iteration
        println("Current Lap: $(lapStatus.currentLap), It: $(lapStatus.currentIt)")

        i                   = lapStatus.currentIt       # just to make notation shorter
        zCurr[i,:]          = z_est                     # update state information

        println("State Nr. $i = $z_est")
        println("Coeff Curvature = $(trackCoeff.coeffCurvature)")

        # Find coefficients for cost and constraints
        mpcCoeff    = coeffConstraintCost(oldTraj,lapStatus,mpcCoeff,posInfo,mpcParams)
        println("Found coefficients: mpcCoeff = $mpcCoeff")
        # Solve the MPC problem
        mpcSol      = solveMpcProblem(mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr[i-1,:]',uCurr[i-1,:]')
        # Write in current input information
        uCurr[i,:]  = [mpcSol.a_x mpcSol.d_f]
        println("posInfo = $posInfo")
        #println("trackCoeff = $trackCoeff")
        println("Finished solving, status: $(mpcSol.solverStatus), u = $(uCurr[i,:])")
        # ... and publish data
        cmd = ECU(mpcSol.a_x, mpcSol.d_f)
        publish(pub, cmd)

        # ==================================
        # Lap trigger ======================
        # ==================================
        if posInfo.s_start + posInfo.s <= s_lapTrigger && switchLap      # if we are switching to the next lap...
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
            println("=========================== NEXT LAP ========================== ")
        elseif posInfo.s_start > s_lapTrigger
            switchLap = true
        end

        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end