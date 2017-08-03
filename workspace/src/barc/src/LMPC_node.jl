#!/usr/bin/env julia

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

# This function is called whenever a new state estimate is received.
# It saves this estimate in oldTraj and uses it in the MPC formulation (see in main)
function SE_callback(msg::pos_info,acc_f::Array{Float64},lapStatus::LapStatus,posInfo::PosInfo,mpcSol::MpcSol,oldTraj::OldTrajectory,trackCoeff::TrackCoeff,z_est::Array{Float64,1},x_est::Array{Float64,1})         # update current position and track data
    # update mpc initial condition
    z_est[:]                  = [msg.v_x,msg.v_y,msg.psiDot,msg.epsi,msg.ey,msg.s,acc_f[1]]             # use z_est as pointer
    x_est[:]                  = [msg.x,msg.y,msg.psi,msg.v]
    trackCoeff.coeffCurvature = msg.coeffCurvature

    # check if lap needs to be switched
    if z_est[6] <= lapStatus.s_lapTrigger && lapStatus.switchLap
        oldTraj.idx_end[lapStatus.currentLap] = oldTraj.count[lapStatus.currentLap]
        oldTraj.oldCost[lapStatus.currentLap] = oldTraj.idx_end[lapStatus.currentLap] - oldTraj.idx_start[lapStatus.currentLap]
        lapStatus.currentLap += 1
        lapStatus.nextLap = true
        lapStatus.switchLap = false
    elseif z_est[6] > lapStatus.s_lapTrigger
        lapStatus.switchLap = true
    end

    # save current state in oldTraj
    oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap],:,lapStatus.currentLap] = z_est
    oldTraj.oldInput[oldTraj.count[lapStatus.currentLap],:,lapStatus.currentLap] = [msg.u_a,msg.u_df]
    #oldTraj.oldInput[oldTraj.count[lapStatus.currentLap],:,lapStatus.currentLap] += 0.5*([msg.u_a msg.u_df]-oldTraj.oldInput[oldTraj.count[lapStatus.currentLap]-1,:,lapStatus.currentLap])
    oldTraj.oldTimes[oldTraj.count[lapStatus.currentLap],lapStatus.currentLap] = to_sec(msg.header.stamp)
    oldTraj.count[lapStatus.currentLap] += 1

    # if necessary: append to end of previous lap
    if lapStatus.currentLap > 1 && z_est[6] < 18.0
        oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap-1],:,lapStatus.currentLap-1] = z_est
        oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap-1],6,lapStatus.currentLap-1] += posInfo.s_target
        oldTraj.oldInput[oldTraj.count[lapStatus.currentLap-1],:,lapStatus.currentLap-1] = [msg.u_a,msg.u_df]
        #oldTraj.oldInput[oldTraj.count[lapStatus.currentLap-1],:,lapStatus.currentLap-1] += 0.5*([msg.u_a msg.u_df]-oldTraj.oldInput[oldTraj.count[lapStatus.currentLap-1]-1,:,lapStatus.currentLap-1])
        oldTraj.oldTimes[oldTraj.count[lapStatus.currentLap-1],lapStatus.currentLap-1] = to_sec(msg.header.stamp)
        oldTraj.count[lapStatus.currentLap-1] += 1
    end

    #if necessary: append to beginning of next lap
    if z_est[6] > posInfo.s_target - 18.0
        oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap+1],:,lapStatus.currentLap+1] = z_est
        oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap+1],6,lapStatus.currentLap+1] -= posInfo.s_target
        oldTraj.oldInput[oldTraj.count[lapStatus.currentLap+1],:,lapStatus.currentLap+1] = [msg.u_a,msg.u_df]
        #oldTraj.oldInput[oldTraj.count[lapStatus.currentLap+1],:,lapStatus.currentLap+1] += 0.5*([msg.u_a msg.u_df]-oldTraj.oldInput[oldTraj.count[lapStatus.currentLap+1]-1,:,lapStatus.currentLap+1])
        oldTraj.oldTimes[oldTraj.count[lapStatus.currentLap+1],lapStatus.currentLap+1] = to_sec(msg.header.stamp)
        oldTraj.count[lapStatus.currentLap+1] += 1
        oldTraj.idx_start[lapStatus.currentLap+1] = oldTraj.count[lapStatus.currentLap+1]
    end
end

# This is the main function, it is called when the node is started.
function main()
    println("Starting LMPC node.")

    buffersize                  = 5000       # size of oldTraj buffers


    # Define and initialize variables
    # ---------------------------------------------------------------
    # General LMPC variables
    oldTraj                     = OldTrajectory()
    posInfo                     = PosInfo()
    mpcCoeff                    = MpcCoeff()
    lapStatus                   = LapStatus(1,1,false,false,0.3)
    mpcSol                      = MpcSol()
    trackCoeff                  = TrackCoeff()      # info about track (at current position, approximated)
    modelParams                 = ModelParams()
    mpcParams                   = MpcParams()
    mpcParams_pF                = MpcParams()       # for 1st lap (path following)

    InitializeParameters(mpcParams,mpcParams_pF,trackCoeff,modelParams,posInfo,oldTraj,mpcCoeff,lapStatus,buffersize)
    mdl    = MpcModel(mpcParams,mpcCoeff,modelParams,trackCoeff)
    mdl_pF = MpcModel_pF(mpcParams_pF,modelParams,trackCoeff)

    max_N = max(mpcParams.N,mpcParams_pF.N)
    # ROS-specific variables
    z_est                       = zeros(7)          # this is a buffer that saves current state information (xDot, yDot, psiDot, ePsi, eY, s)
    x_est                       = zeros(4)          # this is a buffer that saves further state information (x, y, psi, v)
    coeffX                      = zeros(9)          # buffer for coeffX (only logging)
    coeffY                      = zeros(9)          # buffer for coeffY (only logging)
    cmd                         = ECU()             # command type
    coeffCurvature_update       = zeros(trackCoeff.nPolyCurvature+1)

    # Logging variables
    log_coeff_Cost              = NaN*ones(mpcCoeff.order+1,2,10000)
    log_coeff_Const             = NaN*ones(mpcCoeff.order+1,2,5,10000)
    log_sol_z                   = NaN*ones(max_N+1,7,10000)
    log_sol_u                   = NaN*ones(max_N,2,10000)
    log_curv                    = zeros(10000,trackCoeff.nPolyCurvature+1)
    log_state_x                 = zeros(10000,4)
    log_coeffX                  = zeros(10000,9)
    log_coeffY                  = zeros(10000,9)
    log_t                       = zeros(10000,1)
    log_state                   = zeros(10000,7)
    log_cost                    = zeros(10000,6)
    log_c_Vx                    = zeros(10000,3)
    log_c_Vy                    = zeros(10000,4)
    log_c_Psi                   = zeros(10000,3)
    log_cmd                     = zeros(10000,2)
    log_step_diff               = zeros(10000,5)
    log_t_solv                  = zeros(10000)
    log_sol_status              = Array(Symbol,10000)
    
    acc_f = [0.0]

    # Initialize ROS node and topics
    init_node("mpc_traj")
    loop_rate = Rate(1/modelParams.dt)
    pub = Publisher("ecu", ECU, queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
    # The subscriber passes arguments (coeffCurvature and z_est) which are updated by the callback function:
    s1 = Subscriber("pos_info", pos_info, SE_callback, (acc_f,lapStatus,posInfo,mpcSol,oldTraj,trackCoeff,z_est,x_est),queue_size=50)::RobotOS.Subscriber{barc.msg.pos_info}
    # Note: Choose queue size long enough so that no pos_info packets get lost! They are important for system ID!

    run_id = get_param("run_id")
    println("Finished initialization.")
    
    # buffer in current lap
    zCurr                       = zeros(10000,7)    # contains state information in current lap (max. 10'000 steps)
    uCurr                       = zeros(10000,2)    # contains input information
    step_diff                   = zeros(5)

    # Specific initializations:
    lapStatus.currentLap    = 1
    lapStatus.currentIt     = 1
    posInfo.s_target        = 19.11#19.14#17.94#17.76#24.0
    k                       = 0                       # overall counter for logging
    
    mpcSol.z = zeros(11,4)
    mpcSol.u = zeros(10,2)
    mpcSol.a_x = 0
    mpcSol.d_f = 0

    #mpcCoeff.c_Psi = [-0.26682109207165566,-0.013445078992161885,1.2389672517023724]
    #mpcCoeff.c_Psi = [-0.3747957571478858,-0.005013036784512181,5.068342163488241]
    #mpcCoeff.c_Vy  = [-0.006633028965076818,-0.02997779668710061,0.005781203137095575,0.10642934131787765]
    #mpcCoeff.c_Vy  = [0.002968102163011754,-0.09886540158694888,0.012234790760745129,1.099308717654053]
    
    # Precompile coeffConstraintCost:
    oldTraj.oldTraj[1:buffersize,6,1] = linspace(0,posInfo.s_target,buffersize)
    oldTraj.oldTraj[1:buffersize,6,2] = linspace(0,posInfo.s_target,buffersize)
    posInfo.s = posInfo.s_target/2
    lapStatus.currentLap = 3
    oldTraj.count[3] = 500
    coeffConstraintCost(oldTraj,mpcCoeff,posInfo,mpcParams,lapStatus)
    oldTraj.count[3] = 1
    lapStatus.currentLap = 1
    oldTraj.oldTraj[1:buffersize,6,1] = NaN*ones(buffersize,1)
    oldTraj.oldTraj[1:buffersize,6,2] = NaN*ones(buffersize,1)
    posInfo.s = 0

    uPrev = zeros(10,2)     # saves the last 10 inputs (1 being the most recent one)

    n_pf = 3               # number of first path-following laps (needs to be at least 2)

    acc0 = 0.0
    opt_count = 0

    # Start node
    while ! is_shutdown()
        if z_est[6] > 0         # check if data has been received (s > 0)
            # ============================= PUBLISH COMMANDS =============================
            # This is done at the beginning of the lap because this makes sure that the command is published 0.1s after the state has been received
            # This guarantees a constant publishing frequency of 10 Hz
            # (The state can be predicted by 0.1s)
            cmd.header.stamp = get_rostime()
            # cmd.motor = convert(Float32,mpcSol.a_x)
            # cmd.servo = convert(Float32,mpcSol.d_f)
            publish(pub, cmd)
            # ============================= Initialize iteration parameters =============================
            i                           = lapStatus.currentIt           # current iteration number, just to make notation shorter
            zCurr[i,:]                  = copy(z_est)                   # update state information
            posInfo.s                   = zCurr[i,6]                    # update position info
            #trackCoeff.coeffCurvature   = copy(coeffCurvature_update)

            # ============================= Pre-Logging (before solving) ================================
            log_t[k+1]                  = to_sec(get_rostime())         # time is measured *before* solving (more consistent that way)
            if size(mpcSol.z,2) == 4                                    # find 1-step-error
                step_diff = ([mpcSol.z[2,4], 0, 0, mpcSol.z[2,3], mpcSol.z[2,2]]-[norm(zCurr[i,1:2]), 0, 0, zCurr[i,4], zCurr[i,5]])
            else
                step_diff = (mpcSol.z[2,1:5][:]-zCurr[i,1:5][:])
            end
            log_step_diff[k+1,:]          = step_diff

            # ======================================= Lap trigger =======================================
            if lapStatus.nextLap                # if we are switching to the next lap...
                println("Finishing one lap at iteration ",i)
                # Important: lapStatus.currentIt is now the number of points up to s > s_target -> -1 in saveOldTraj
                zCurr[1,:] = zCurr[i,:]         # copy current state
                i                     = 1
                lapStatus.currentIt   = 1       # reset current iteration
                lapStatus.nextLap = false

                # Set warm start for new solution (because s shifted by s_target)
                if lapStatus.currentLap <= n_pf
                    setvalue(mdl_pF.z_Ol[:,1],mpcSol.z[:,1]-posInfo.s_target)
                elseif lapStatus.currentLap == n_pf+1
                    setvalue(mdl.z_Ol[:,1],mpcSol.z[1:mpcParams.N+1,4])
                    setvalue(mdl.z_Ol[:,6],mpcSol.z[1:mpcParams.N+1,1]-posInfo.s_target)
                    setvalue(mdl.z_Ol[:,5],mpcSol.z[1:mpcParams.N+1,2])
                    setvalue(mdl.z_Ol[:,4],mpcSol.z[1:mpcParams.N+1,3])
                    setvalue(mdl.u_Ol,mpcSol.u[1:mpcParams.N,:])
                elseif lapStatus.currentLap > n_pf+1
                    setvalue(mdl.z_Ol[:,6],mpcSol.z[:,6]-posInfo.s_target)
                end
            end

            #  ======================================= Calculate input =======================================
            #println("*** NEW ITERATION # ",i," ***")
            println("Current Lap: ", lapStatus.currentLap, ", It: ", lapStatus.currentIt)
            #println("State Nr. ", i, "    = ", z_est)
            #println("s               = $(posInfo.s)")
            #println("s_total         = $(posInfo.s%posInfo.s_target)")

            # Find coefficients for cost and constraints
            if lapStatus.currentLap > n_pf
                tic()
                coeffConstraintCost(oldTraj,mpcCoeff,posInfo,mpcParams,lapStatus)
                tt = toq()
                println("Finished coefficients, t = ",tt," s")
            end

            #println("Starting solving.")
            # Solve the MPC problem
            tic()
            if lapStatus.currentLap <= n_pf
                z_pf = [zCurr[i,6],zCurr[i,5],zCurr[i,4],norm(zCurr[i,1:2]),acc0]        # use kinematic model and its states
                solveMpcProblem_pathFollow(mdl_pF,mpcSol,mpcParams_pF,trackCoeff,posInfo,modelParams,z_pf,uPrev)
                acc_f[1] = mpcSol.z[1,5]
                acc0 = mpcSol.z[2,5]
            else                        # otherwise: use system-ID-model
                #mpcCoeff.c_Vx[3] = max(mpcCoeff.c_Vx[3],0.1)
                zCurr[i,7] = acc0
                solveMpcProblem(mdl,mpcSol,mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr[i,:]',uPrev)
                acc0 = mpcSol.z[2,7]
                acc_f[1] = mpcSol.z[1,7]
            end
            log_t_solv[k+1] = toq()

            # Send command immediately, only if it is optimal!
            #if mpcSol.solverStatus == :Optimal
            #    opt_count = 0
            #else                        # otherwise use the last optimal input
                #mpcSol.a_x = uPrev[1,1]
                #mpcSol.d_f = uPrev[1,2]
                #opt_count += 1
                if opt_count >= 5
                    warn("No optimal solution for $opt_count iterations.")
                end
            #end

            #cmd.header.stamp = get_rostime()
            cmd.motor = convert(Float32,mpcSol.a_x)
            cmd.servo = convert(Float32,mpcSol.d_f)
            #publish(pub, cmd)

            # Write current input information
            uCurr[i,:] = [mpcSol.a_x mpcSol.d_f]
            zCurr[i,6] = posInfo.s%posInfo.s_target   # save absolute position in s (for oldTrajectory)

            uPrev = circshift(uPrev,1)
            uPrev[1,:] = uCurr[i,:]
            #println("Finished solving, status: $(mpcSol.solverStatus), u = $(uCurr[i,:]), t = $(log_t_solv[k+1]) s")

            # Logging
            # ---------------------------
            k = k + 1       # counter
            log_sol_status[k]       = mpcSol.solverStatus
            log_state[k,:]          = zCurr[i,:]
            log_cmd[k+1,:]          = uCurr[i,:]                    # the command is going to be pubished in the next iteration
            log_coeff_Cost[:,:,k]   = mpcCoeff.coeffCost
            log_coeff_Const[:,:,:,k] = mpcCoeff.coeffConst
            log_cost[k,:]           = mpcSol.cost
            log_curv[k,:]           = trackCoeff.coeffCurvature
            log_state_x[k,:]        = x_est
            log_c_Vx[k,:]           = mpcCoeff.c_Vx
            log_c_Vy[k,:]           = mpcCoeff.c_Vy
            log_c_Psi[k,:]          = mpcCoeff.c_Psi
            if size(mpcSol.z,2) == 5
                log_sol_z[1:mpcParams_pF.N+1,1:5,k]     = mpcSol.z        # only 4 states during path following mode (first 2 laps)
                log_sol_u[1:mpcParams_pF.N,:,k]         = mpcSol.u
            else
                log_sol_z[1:mpcParams.N+1,1:7,k]        = mpcSol.z
                log_sol_u[1:mpcParams.N,:,k]            = mpcSol.u
            end

            # Count one up:
            lapStatus.currentIt += 1
        else
            println("No estimation data received!")
        end
        rossleep(loop_rate)
    end
    # Save simulation data to file

    log_path = "$(homedir())/simulations/output-LMPC-$(run_id[1:4]).jld"
    if isfile(log_path)
        log_path = "$(homedir())/simulations/output-LMPC-$(run_id[1:4])-2.jld"
        warn("Warning: File already exists.")
    end
    save(log_path,"oldTraj",oldTraj,"state",log_state[1:k,:],"t",log_t[1:k],"sol_z",log_sol_z[:,:,1:k],"sol_u",log_sol_u[:,:,1:k],
                    "cost",log_cost[1:k,:],"curv",log_curv[1:k,:],"coeffCost",log_coeff_Cost,"coeffConst",log_coeff_Const,
                    "x_est",log_state_x[1:k,:],"coeffX",log_coeffX[1:k,:],"coeffY",log_coeffY[1:k,:],"c_Vx",log_c_Vx[1:k,:],
                    "c_Vy",log_c_Vy[1:k,:],"c_Psi",log_c_Psi[1:k,:],"cmd",log_cmd[1:k,:],"step_diff",log_step_diff[1:k,:],
                    "t_solv",log_t_solv[1:k],"sol_status",log_sol_status[1:k])
    println("Exiting LMPC node. Saved data to $log_path.")

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

# zCurr[1] = v_x
# zCurr[2] = v_y
# zCurr[3] = psiDot
# zCurr[4] = ePsi
# zCurr[5] = eY
# zCurr[6] = s
