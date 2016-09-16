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
#using JLD

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
posInfo.s_target            = 10.281192
coeffCurvature_update       = 0             # use extra update variables so that they're not suddenly changed within functions
s_start_update              = 0

function SE_callback(msg::pos_info)         # update current position and track data
    # update mpc initial condition
    global z_est, posInfo, coeffCurvature_update, s_start_update
    z_est                     = [msg.s msg.ey msg.epsi msg.v]
    s_start_update            = msg.s_start
    coeffCurvature_update     = msg.coeffCurvature
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

    buffersize                  = 700

    # Create data to be saved
    # save_oldTraj = zeros(buffersize,4,2,4)  # max. 4 laps

    # Define and initialize variables
    global mpcParams, trackCoeff, modelParams, z_est, posInfo, coeffCurvature_update, s_start_update
    oldTraj                     = OldTrajectory()
    mpcCoeff                    = MpcCoeff()
    lapStatus                   = LapStatus(1,1)
    mpcSol                      = MpcSol()

    mpcParams.QderivZ           = 0.0*[1 1 1 1]             # cost matrix for derivative cost of states
    mpcParams.QderivU           = 0.1*[1 1]                 # cost matrix for derivative cost of inputs
    mpcParams.R                 = 0.0*[1 1]                 # cost matrix for control inputs
    mpcParams.Q                 = [0.0 10.0 1.0 1.0]        # put weights on ey, epsi and v

    oldTraj.oldTraj             = zeros(buffersize,4,2)
    oldTraj.oldInput            = zeros(buffersize,2,2)

    mpcCoeff.coeffCost          = Float64[]
    mpcCoeff.coeffConst         = Float64[]
    mpcCoeff.order              = 5
    mpcCoeff.pLength            = 4*mpcParams.N        # small values here may lead to numerical problems since the functions are only approximated in a short horizon

    lapStatus.currentLap        = 1         # initialize lap number
    lapStatus.currentIt         = 0         # current iteration in lap

    # Lap parameters
    switchLap               = false     # initialize lap lap trigger
    s_lapTrigger            = 0.3       # next lap is triggered in the interval s_start in [0,s_lapTrigger]
    
    # buffer in current lap
    zCurr       = zeros(10000,4)    # contains state information in current Lap (max. 10'000 steps)
    uCurr       = zeros(10000,2)    # contains input information

    # Precompile functions by running them once:
    solve(mdl)
    coeffConstraintCost(oldTraj,lapStatus,mpcCoeff,posInfo,mpcParams)

    println("Starting loop")
    while ! is_shutdown()
        if z_est[1] > 0         # check if data has been received (s > 0)            

            # ============================= Initialize iteration parameters =============================
            lapStatus.currentIt += 1                            # count iteration

            i                   = lapStatus.currentIt           # current iteration number, just to make notation shorter
            zCurr[i,:]          = z_est                         # update state information
            posInfo.s           = z_est[1]                      # update position info
            posInfo.s_start     = s_start_update
            trackCoeff.coeffCurvature = coeffCurvature_update


            # ======================================= Lap trigger =======================================
            if (posInfo.s_start + posInfo.s)%posInfo.s_target <= s_lapTrigger && switchLap      # if we are switching to the next lap...
                # ... then select and save data
                tic()
                zCurr_export    = cat(1,zCurr[1:i-1,:], [zCurr[i-1,1]+collect(1:buffersize-i+1)*dt*zCurr[i-1,4] ones(buffersize-i+1,1)*zCurr[i-1,2:4]])
                uCurr_export    = cat(1,uCurr[1:i-1,:], zeros(buffersize-i+1,2))
                costLap         = lapStatus.currentIt               # the cost of the current lap is the time it took to reach the finish line
                # Save all data in oldTrajectory:
                if lapStatus.currentLap == 1                        # if it's the first lap
                    oldTraj.oldTraj[:,:,1]  = zCurr_export         # ... just save everything
                    oldTraj.oldInput[:,:,1] = uCurr_export
                    oldTraj.oldTraj[:,:,2]  = zCurr_export
                    oldTraj.oldInput[:,:,2] = uCurr_export
                    oldTraj.oldCost = [costLap,costLap]
                else                                                # idea: always copy the new trajectory in the first array!
                    if oldTraj.oldCost[1] < oldTraj.oldCost[2]      # if the first old traj is better than the second
                        oldTraj.oldTraj[:,:,2]  = oldTraj.oldTraj[:,:,1]    # ... copy the first in the second
                        oldTraj.oldInput[:,:,2] = oldTraj.oldInput[:,:,1]   # ... same for the input
                        oldTraj.oldCost[2] = oldTraj.oldCost[1]
                    end
                    oldTraj.oldTraj[:,:,1]  = zCurr_export                 # ... and write the new traj in the first
                    oldTraj.oldInput[:,:,1] = uCurr_export
                    oldTraj.oldCost[1] = costLap
                end
                #println(size(save_oldTraj))
                #println(size(oldTraj.oldTraj))
                #save_oldTraj[:,:,:,lapStatus.currentLap] = oldTraj.oldTraj[:,:,:]
                zCurr[1,:] = zCurr[i,:]         # reset counter to 1 and set current state
                uCurr[1,:] = uCurr[i+1,:]       # ... and input
                i                     = 1       
                lapStatus.currentLap += 1       # start next lap
                lapStatus.currentIt   = 1       # reset current iteration
                switchLap = false
                tt = toq()
                println("======================================== NEXT LAP ========================================")
                println("oldTraj.oldTraj[:,1,1]:")
                println(oldTraj.oldTraj[:,1,1])
            elseif (posInfo.s_start+posInfo.s)%posInfo.s_target > s_lapTrigger
                switchLap = true
            end


            #  ======================================= Calculate input =======================================
            println("======================================== NEW ITERATION # $i ========================================")
            println("Current Lap: $(lapStatus.currentLap), It: $(lapStatus.currentIt)")
            println("State Nr. $i    = $z_est")
            println("Coeff Curvature = $(trackCoeff.coeffCurvature)")
            println("posInfo         = $posInfo")
            println("s               = $(posInfo.s)")
            println("s_start         = $(posInfo.s_start)")
            println("s_total         = $((posInfo.s+posInfo.s_start)%posInfo.s_target)")

            # Find coefficients for cost and constraints
            tic()
            mpcCoeff    = coeffConstraintCost(oldTraj,lapStatus,mpcCoeff,posInfo,mpcParams)
            tt = toq()
            println("Finished coefficients, t = $tt s")
            #println("Found coefficients: mpcCoeff = $mpcCoeff")
            # Solve the MPC problem
            tic()
            mpcSol      = solveMpcProblem(mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr[i,:]',uCurr[i,:]')
            tt = toq()
            # Write in current input information
            uCurr[i+1,:]  = [mpcSol.a_x mpcSol.d_f]
            #println("trackCoeff = $trackCoeff")
            println("Finished solving, status: $(mpcSol.solverStatus), u = $(uCurr[i,:]), t = $tt s")
            # ... and publish data
            cmd = ECU(mpcSol.a_x, mpcSol.d_f)
            publish(pub, cmd)

            zCurr[i,1] = (posInfo.s_start + posInfo.s)%posInfo.s_target   # save absolute position in s (for oldTrajectory)
            println("\n")
        
        else
            println("No estimation data received!")
        end
        rossleep(loop_rate)
    end
    # Save simulation data to file
    # log_path = "$(homedir())/simulations/LMPC_output.jld"
    # save(log_path,"oldTraj",save_oldTraj)
    # println("Exiting LMPC node. Saved data.")
end

if ! isinteractive()
    main()
end