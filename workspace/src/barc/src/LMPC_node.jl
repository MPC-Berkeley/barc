#!/usr/bin/env julia

using RobotOS
@rosimport barc.msg: ECU, pos_info, Encoder, Ultrasound, Logging
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using JuMP
using Ipopt
using JLD

include("LMPC_lib/classes.jl")
include("LMPC_lib/MPC_models.jl")
include("LMPC_lib/coeffConstraintCost.jl")
include("LMPC_lib/solveMpcProblem.jl")
include("LMPC_lib/functions.jl")

function SE_callback(msg::pos_info,s_start_update::Array{Float64},coeffCurvature_update::Array{Float64,1},z_est::Array{Float64,1},x_est::Array{Float64,1},
                        coeffX::Array{Float64,1},coeffY::Array{Float64,1})         # update current position and track data
    # update mpc initial condition
    z_est[:]                  = [msg.v_x,msg.v_y,msg.psiDot,msg.epsi,msg.ey,msg.s]             # use z_est as pointer
    s_start_update[1]         = msg.s_start
    coeffCurvature_update[:]  = msg.coeffCurvature
    x_est[:]                  = [msg.x,msg.y,msg.psi,msg.v]
    coeffX[:]                 = msg.coeffX
    coeffY[:]                 = msg.coeffY
end

function main()
    println("Starting LMPC node.")

    buffersize                  = 1000       # size of oldTraj buffers

    # Define and initialize variables
    # ---------------------------------------------------------------
    # General LMPC variables
    oldTraj                     = OldTrajectory()
    posInfo                     = PosInfo()
    mpcCoeff                    = MpcCoeff()
    lapStatus                   = LapStatus(1,1)
    mpcSol                      = MpcSol()
    trackCoeff                  = TrackCoeff()      # info about track (at current position, approximated)
    modelParams                 = ModelParams()
    mpcParams                   = MpcParams()
    mpcParams_pF                = MpcParams()       # for 1st lap (path following)

    z_Init    = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]      # xDot needs to be > 0
    z_Init_pF = zeros(4)

    InitializeParameters(mpcParams,mpcParams_pF,trackCoeff,modelParams,posInfo,oldTraj,mpcCoeff,lapStatus,buffersize)
    mdl    = MpcModel(mpcParams,mpcCoeff,modelParams,trackCoeff,z_Init)
    mdl_pF = MpcModel_pF(mpcParams_pF,modelParams,trackCoeff,z_Init_pF)

    z_ID = zeros(50,6)
    u_ID = zeros(50,2)

    # ROS-specific variables
    z_est                       = zeros(6)          # this is a buffer that saves current state information (xDot, yDot, psiDot, ePsi, eY, s)
    x_est                       = zeros(4)          # this is a buffer that saves further state information (x, y, psi, v)
    coeffX                      = zeros(9)          # buffer for coeffX (only logging)
    coeffY                      = zeros(9)          # buffer for coeffY (only logging)
    s_start_update              = [0.0]             # buffer for s_start
    cmd                         = ECU(0.0,0.0)      # command type
    coeffCurvature_update       = zeros(trackCoeff.nPolyCurvature+1)

    # Logging variables
    log_coeff_Cost              = zeros(mpcCoeff.order+1,2,10000)
    log_coeff_Const             = zeros(mpcCoeff.order+1,2,5,10000)
    log_sol_z                   = zeros(mpcParams.N+1,6,10000)
    log_sol_u                   = zeros(mpcParams.N,2,10000)
    log_curv                    = zeros(10000,trackCoeff.nPolyCurvature+1)
    log_s_start                 = zeros(10000)
    log_state_x                 = zeros(10000,4)
    log_coeffX                  = zeros(10000,9)
    log_coeffY                  = zeros(10000,9)
    log_t                       = zeros(10000,1)
    log_state                   = zeros(10000,6)
    log_cost                    = zeros(10000,6)
    log_c_Vx                    = zeros(10000,3)
    log_c_Vy                    = zeros(10000,4)
    log_c_Psi                   = zeros(10000,3)
    log_cmd                     = zeros(10000,2)
    log_oldTraj                 = zeros(buffersize,6,2,20)  # max. 20 laps
    
    # Initialize ROS node and topics
    init_node("mpc_traj")
    loop_rate = Rate(10)
    pub                         = Publisher("ecu", ECU, queue_size=1)
    # The subscriber passes arguments (s_start, coeffCurvature and z_est) which are updated by the callback function:
    s1                          = Subscriber("pos_info", pos_info, SE_callback, (s_start_update,coeffCurvature_update,z_est,x_est,coeffX,coeffY,),queue_size=1)

    println("Finished initialization.")
    # Lap parameters
    switchLap                   = false     # initialize lap lap trigger
    s_lapTrigger                = 0.3       # next lap is triggered in the interval s_start in [0,s_lapTrigger]
    
    # buffer in current lap
    zCurr                       = zeros(10000,6)    # contains state information in current lap (max. 10'000 steps)
    uCurr                       = zeros(10000,2)    # contains input information
    u_final                     = zeros(2)          # contains last input of one lap

    # Specific initializations:
    lapStatus.currentLap    = 1
    lapStatus.currentIt     = 1
    posInfo.s_target        = 12.0#24.0
    k                       = 0                       # overall counter for logging
    
    # Precompile coeffConstraintCost:
    oldTraj.oldTraj[1:buffersize,6,1] = linspace(0,posInfo.s_target,buffersize)
    oldTraj.oldTraj[1:buffersize,6,2] = linspace(0,posInfo.s_target,buffersize)
    posInfo.s = posInfo.s_target/2
    posInfo.s_start = 0
    coeffConstraintCost(oldTraj,mpcCoeff,posInfo,mpcParams,z_ID,u_ID,lapStatus)
    oldTraj.oldTraj[1:buffersize,6,1] = zeros(buffersize,1)
    oldTraj.oldTraj[1:buffersize,6,2] = zeros(buffersize,1)
    posInfo.s = 0
    posInfo.s_start = 0

    # Start node
    while ! is_shutdown()
        if z_est[6] > 0         # check if data has been received (s > 0)

            # ============================= PUBLISH COMMANDS =============================
            # this is done at the beginning of the lap because this makes sure that the command is published 0.1s after the state has been received
            # the state is predicted by 0.1s
            cmd.motor = mpcSol.a_x
            cmd.servo = mpcSol.d_f
            publish(pub, cmd)        

            # ============================= Initialize iteration parameters =============================
            i                           = lapStatus.currentIt           # current iteration number, just to make notation shorter
            log_t[k+1]                  = time()
            zCurr[i,:]                  = copy(z_est)                   # update state information (actually predicted by Kalman filter!)
            posInfo.s                   = zCurr[i,6]                    # update position info
            posInfo.s_start             = copy(s_start_update[1])       # use a copy so s_start is not updated during one iteration
            trackCoeff.coeffCurvature   = copy(coeffCurvature_update)

            # ======================================= Lap trigger =======================================
            if (posInfo.s_start + posInfo.s)%posInfo.s_target <= s_lapTrigger && switchLap      # if we are switching to the next lap...
                println("Finishing one lap at iteration $i")
                println("current state:  $(zCurr[i,:])")
                println("previous state: $(zCurr[i-1,:])")
                # Important: lapStatus.currentIt is now the number of points up to s > s_target -> -1 in saveOldTraj
                saveOldTraj(oldTraj,zCurr,uCurr,lapStatus,posInfo,buffersize)
                println("oldTraj: $(oldTraj.oldTraj[:,:,1])")
                log_oldTraj[:,:,:,lapStatus.currentLap] = oldTraj.oldTraj[:,:,:]
                zCurr[1,:] = zCurr[i,:]         # copy current state
                u_final    = uCurr[i-1,:]         # ... and input
                i                     = 1
                lapStatus.currentIt   = 1       # reset current iteration
                lapStatus.currentLap += 1       # start next lap
                switchLap = false
            elseif (posInfo.s_start+posInfo.s)%posInfo.s_target > s_lapTrigger
                switchLap = true
            end

            # For System ID: Update last 50 measurements
            z_ID = circshift(z_ID,-1)
            u_ID = circshift(u_ID,-1)
            z_ID[end,:] = zCurr[i,:]
            u_ID[end,:] = uCurr[i,:]

            #  ======================================= Calculate input =======================================
            println("======================================== NEW ITERATION # $i ========================================")
            println("Current Lap: $(lapStatus.currentLap), It: $(lapStatus.currentIt)")
            println("State Nr. $i    = $z_est")
            println("s               = $(posInfo.s)")
            println("s_start         = $(posInfo.s_start)")
            println("s_total         = $((posInfo.s+posInfo.s_start)%posInfo.s_target)")

            # Find coefficients for cost and constraints
            if lapStatus.currentLap > 2
                tic()
                coeffConstraintCost(oldTraj,mpcCoeff,posInfo,mpcParams,z_ID,u_ID,lapStatus)
                tt = toq()
                println("Finished coefficients, t = $tt s")
            end

            # Find last inputs u_i-1
            if i>1                      # last u is needed for smooth MPC solutions (for derivative of u)
                last_u = uCurr[i-1,:]
            else
                last_u = u_final
            end

            # Solve the MPC problem
            tic()
            if lapStatus.currentLap <= 2
                z_pf = [zCurr[i,6],zCurr[i,5],zCurr[i,4],zCurr[i,1]]        # use kinematic model and its states
                solveMpcProblem_pathFollow(mdl_pF,mpcSol,mpcParams_pF,trackCoeff,posInfo,modelParams,z_pf,last_u')
            else                        # otherwise: use system-ID-model
                solveMpcProblem(mdl,mpcSol,mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr[i,:]',last_u')
            end
            tt = toq()

            # Write current input information
            uCurr[i,:]  = [mpcSol.a_x mpcSol.d_f]
            zCurr[i,6] = (posInfo.s_start + posInfo.s)%posInfo.s_target   # save absolute position in s (for oldTrajectory)

            println("Finished solving, status: $(mpcSol.solverStatus), u = $(uCurr[i,:]), t = $tt s")

            # append new states and inputs to old trajectory
            oldTraj.oldTraj[oldTraj.oldCost[1]+oldTraj.prebuf+i,:,1] = zCurr[i,:]
            oldTraj.oldTraj[oldTraj.oldCost[1]+oldTraj.prebuf+i,6,1] += posInfo.s_target
            oldTraj.oldInput[oldTraj.oldCost[1]+oldTraj.prebuf+i,:,1] = uCurr[i,:]
            if lapStatus.currentLap==3     # if its the third lap, append to both old trajectories! (since both are the same)
                oldTraj.oldTraj[oldTraj.oldCost[1]+oldTraj.prebuf+i,:,2] = zCurr[i,:]
                oldTraj.oldTraj[oldTraj.oldCost[1]+oldTraj.prebuf+i,6,2] += posInfo.s_target
                oldTraj.oldInput[oldTraj.oldCost[1]+oldTraj.prebuf+i,:,2] = uCurr[i,:]
            end

            # Logging
            # ---------------------------
            k = k + 1       # counter
            log_state[k,:]          = zCurr[i,:]
            log_cmd[k+1,:]          = uCurr[i,:]                    # the command is going to be pubished in the next iteration
            log_sol_u[:,:,k]        = mpcSol.u
            log_coeff_Cost[:,:,k]   = mpcCoeff.coeffCost
            log_coeff_Const[:,:,:,k] = mpcCoeff.coeffConst
            log_cost[k,:]           = mpcSol.cost
            log_curv[k,:]           = trackCoeff.coeffCurvature
            log_s_start[k]          = posInfo.s_start
            log_state_x[k,:]        = x_est
            log_c_Vx[k,:]           = mpcCoeff.c_Vx
            log_c_Vy[k,:]           = mpcCoeff.c_Vy
            log_c_Psi[k,:]          = mpcCoeff.c_Psi
            #log_mpcCoeff[k]         = copy(mpcCoeff)
            if lapStatus.currentLap <= 2
                log_sol_z[:,1:4,k]        = mpcSol.z        # only 4 states during path following mode (first 2 laps)
            else
                log_sol_z[:,1:6,k]        = mpcSol.z
            end

            # Count one up:
            lapStatus.currentIt += 1
        else
            println("No estimation data received!")
        end
        rossleep(loop_rate)
    end
    # Save simulation data to file

    log_path = "$(homedir())/simulations/output_LMPC.jld"
    save(log_path,"oldTraj",log_oldTraj,"state",log_state[1:k,:],"t",log_t[1:k],"sol_z",log_sol_z[:,:,1:k],"sol_u",log_sol_u[:,:,1:k],
                    "cost",log_cost[1:k,:],"curv",log_curv[1:k,:],"coeffCost",log_coeff_Cost,"coeffConst",log_coeff_Const,
                    "s_start",log_s_start[1:k],"x_est",log_state_x[1:k,:],"coeffX",log_coeffX[1:k,:],"coeffY",log_coeffY[1:k,:],"c_Vx",log_c_Vx[1:k,:],
                    "c_Vy",log_c_Vy[1:k,:],"c_Psi",log_c_Psi[1:k,:],"cmd",log_cmd[1:k,:])
    println("Exiting LMPC node. Saved data.")

end

if ! isinteractive()
    main()
end

# Sequence within one iteration:
# 1. Publish commands from last iteration (because the car is in real *now* where we thought it was before (predicted state))
# 2. Receive new state information
# 3. Check if we've crossed the finish line and if we have, switch lap number and save old trajectories
# 4. (If in 3rd lap): Calculate coefficients
# 5. Calculate MPC commands (depending on lap) which are going to be published in the next iteration
# 6. (Do some logging)


# Definitions of variables:
# zCurr contains all state information from the beginning of the lap (first s >= 0) to the current state i
# uCurr -> same as zCurr for inputs
# generally: zCurr[i+1] = f(zCurr[i],uCurr[i])