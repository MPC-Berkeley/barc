#!/usr/bin/env julia

using RobotOS
@rosimport barc.msg: ECU, pos_info
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
    obstacle                    = Obstacle()
    selectedStates              = SelectedStates()
    oldSS                       = SafeSetData()

    InitializeParameters(mpcParams,mpcParams_pF,trackCoeff,modelParams,posInfo,oldTraj,mpcCoeff,lapStatus,buffersize,obstacle,selectedStates,oldSS)

    mdl_pF       = MpcModel_pF(mpcParams_pF,modelParams,trackCoeff)

    if selectedStates.version == true
        mdl          = MpcModel(mpcParams,mpcCoeff,modelParams,trackCoeff)  
    elseif selectedStates.version == false
        mdl_convhull = MpcModel_convhull(mpcParams,mpcCoeff,modelParams,trackCoeff,selectedStates)
    end

    
   
    #mdl_test     = MpcModel_test(mpcParams,mpcCoeff,modelParams,trackCoeff,selectedStates)
    mdl_obstacle = MpcModel_obstacle(mpcParams,mpcCoeff,modelParams,trackCoeff,selectedStates,obstacle)


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
    log_final_counter           = zeros(30)

    selStates_log               = zeros(selectedStates.Nl*selectedStates.Np,6,buffersize,30)   #array to log the selected states in every iteration of every lap
    statesCost_log              = zeros(selectedStates.Nl*selectedStates.Np,buffersize,30)     #array to log the selected states' costs in every iteration of every lap
    log_predicted_sol           = zeros(mpcParams.N+1,7,buffersize,30)
    log_predicted_input         = zeros(mpcParams.N,2,buffersize,30)
    log_onestep                 = zeros(buffersize,6,30)
    log_epsalpha                = zeros(6,buffersize,30)
    log_cvx                     = zeros(buffersize,3,30)
    log_cvy                     = zeros(buffersize,4,30)
    log_cpsi                    = zeros(buffersize,3,30)
    log_input                   = zeros(buffersize,2,30)
    log_status                  = Array(Symbol,buffersize,30)
    log_mpcCost                 = zeros(buffersize,6,30)
    log_mpcCostSlack            = zeros(buffersize,6,30)
    log_obs                     = zeros(buffersize,3,obstacle.n_obs,30)


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

    obs_curr                    = zeros(buffersize,3,obstacle.n_obs)::Array{Float64,3}      # info about the obstacle in the current lap


    # Specific initializations:
    lapStatus.currentLap    = 1
    lapStatus.currentIt     = 1
    posInfo.s_target        = 19.11 #17.91 #19.14#17.94#17.76#24.0
    k                       = 0                       # overall counter for logging
    
    mpcSol.z = zeros(11,4)
    mpcSol.u = zeros(10,2)
    mpcSol.a_x = 0
    mpcSol.d_f = 0
    
    # Precompile coeffConstraintCost:
    oldTraj.oldTraj[1:buffersize,6,1] = linspace(0,posInfo.s_target,buffersize)
    oldTraj.oldTraj[1:buffersize,6,2] = linspace(0,posInfo.s_target,buffersize)
    oldTraj.oldTraj[1:buffersize,6,3] = linspace(0,posInfo.s_target,buffersize)


    oldSS.oldSS[1:buffersize,6,1]     = linspace(0.1,posInfo.s_target,buffersize)
    oldSS.oldSS[1:buffersize,6,2]     = linspace(0.1,posInfo.s_target,buffersize)
    oldSS.oldSS[1:buffersize,6,3]     = linspace(0.1,posInfo.s_target,buffersize)

    posInfo.s = posInfo.s_target/2
    lapStatus.currentLap = 4
    oldTraj.count[4] = 500
    coeffConstraintCost(oldTraj,mpcCoeff,posInfo,mpcParams,lapStatus,selectedStates,oldSS,obs_curr[1,:,:],obstacle)
    oldTraj.count[4] = 1
    lapStatus.currentLap = 1
    oldTraj.oldTraj[1:buffersize,6,1] = NaN*ones(buffersize,1)
    oldTraj.oldTraj[1:buffersize,6,2] = NaN*ones(buffersize,1)
    oldTraj.oldTraj[1:buffersize,6,3] = NaN*ones(buffersize,1)

    posInfo.s = 0

    #selectedStates.selStates    = zeros(selectedStates.Nl*selectedStates.Np,6)  
    #selectedStates.statesCost   = zeros(selectedStates.Nl*selectedStates.Np)
    oldSS.oldSS                 = NaN*ones(buffersize,7,30)


    uPrev = zeros(10,2)     # saves the last 10 inputs (1 being the most recent one)

    n_pf = 3               # number of first path-following laps (needs to be at least 2)

    acc0 = 0.0
    opt_count = 0

    same_sPF   = 0
    same_sLMPC = 0

    #### Set initial conditions on the obstacles

    obs_curr[1,1,:] = obstacle.s_obs_init
    obs_curr[1,2,:] = obstacle.ey_obs_init
    obs_curr[1,3,:] = obstacle.v_obs_init

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

            # Check if and how many times the states are repeated 
            if i>1
                if z_est[6] == zCurr[i-1,6] && lapStatus.currentLap <= n_pf
                    same_sPF +=1
                elseif z_est[6] == zCurr[i-1,6] && lapStatus.currentLap > n_pf
                    same_sLMPC +=1
                end
            end

            zCurr[i,:]                  = copy(z_est)                   # update state information
            posInfo.s                   = zCurr[i,6]                    # update position info
            lap_now                     = lapStatus.currentLap

            #trackCoeff.coeffCurvature   = copy(coeffCurvature_update)

            # ============================= Pre-Logging (before solving) ================================
            log_t[k+1]                  = to_sec(get_rostime())         # time is measured *before* solving (more consistent that way)
            if size(mpcSol.z,2) == 4                                    # find 1-step-error
                step_diff = ([mpcSol.z[2,4], 0, 0, mpcSol.z[2,3], mpcSol.z[2,2]]-[norm(zCurr[i,1:2]), 0, 0, zCurr[i,4], zCurr[i,5]])
            else
                step_diff = (mpcSol.z[2,1:5][:]-zCurr[i,1:5][:])
            end
            log_step_diff[k+1,:]          = step_diff

            if size(mpcSol.z,2) > 5
                
                log_onestep[lapStatus.currentIt,:,lapStatus.currentLap] = abs(mpcSol.z[2,1:6] - zCurr[i,1:6])
                
            end

            # ======================================= Lap trigger =======================================
            if lapStatus.nextLap                # if we are switching to the next lap...
                println("Finishing one lap at iteration ",i)
                # Important: lapStatus.currentIt is now the number of points up to s > s_target -> -1 in saveOldTraj

                log_final_counter[lapStatus.currentLap-1] = k

                oldSS.oldCost[lapStatus.currentLap-1] = lapStatus.currentIt
                cost2target                           = zeros(buffersize) # array containing the cost to arrive from each point of the old trajectory to the target
                #save the terminal cost
                for j = 1:buffersize
                    cost2target[j] = mpcParams.Q_term_cost*(lapStatus.currentIt-j+1)  
                end
                oldSS.cost2target[:,lapStatus.currentLap-1] = cost2target
                                

                if lapStatus.currentLap == obstacle.lap_active            # if its time to put the obstacles in the track
                    obstacle.obstacle_active = true    # tell the system to put the obstacles on the track
                end
                if lapStatus.currentLap > obstacle.lap_active             # initialize current obstacle states with final states from the previous lap
                    obs_curr[1,:,:] = obs_curr[i,:,:]
                end


                zCurr[1,:] = zCurr[i,:]         # copy current state
                i                     = 1
                lapStatus.currentIt   = 1       # reset current iteration
                lapStatus.nextLap = false

                # Set warm start for new solution (because s shifted by s_target)
                if lapStatus.currentLap <= n_pf
                    setvalue(mdl_pF.z_Ol[:,1],mpcSol.z[:,1]-posInfo.s_target)
                elseif lapStatus.currentLap == n_pf+1
                    if selectedStates.version == true
                        setvalue(mdl.z_Ol[1:mpcParams.N,1],mpcSol.z[1:mpcParams.N,4])
                        setvalue(mdl.z_Ol[1:mpcParams.N,6],mpcSol.z[1:mpcParams.N,1]-posInfo.s_target)
                        setvalue(mdl.z_Ol[1:mpcParams.N,5],mpcSol.z[1:mpcParams.N,2])
                        setvalue(mdl.z_Ol[1:mpcParams.N,4],mpcSol.z[1:mpcParams.N,3])
                        setvalue(mdl.u_Ol,mpcSol.u[1:mpcParams.N,:])
                    
                    elseif selectedStates.version == false && obstacle.obstacle_active == false

                        setvalue(mdl_convhull.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,4])
                        setvalue(mdl_convhull.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,4])
                        setvalue(mdl_convhull.z_Ol[1:mpcParams.N,6],mpcSol.z[2:mpcParams.N+1,1]-posInfo.s_target)
                        setvalue(mdl_convhull.z_Ol[mpcParams.N+1,6],mpcSol.z[mpcParams.N+1,1]-posInfo.s_target)
                        setvalue(mdl_convhull.z_Ol[1:mpcParams.N,5],mpcSol.z[2:mpcParams.N+1,2])
                        setvalue(mdl_convhull.z_Ol[mpcParams.N+1,5],mpcSol.z[mpcParams.N+1,2])
                        setvalue(mdl_convhull.z_Ol[1:mpcParams.N,4],mpcSol.z[2:mpcParams.N+1,3])
                        setvalue(mdl_convhull.z_Ol[mpcParams.N+1,4],mpcSol.z[mpcParams.N+1,3])

                        #setvalue(mdl_convhull.z_Ol[:,2],zCurr[1,2]*ones(mpcParams.N+1))
                        #setvalue(mdl_convhull.z_Ol[:,3],zCurr[1,3]*ones(mpcParams.N+1))
                        #setvalue(mdl_convhull.u_Ol,mpcSol.u[1:mpcParams.N,:])
                        setvalue(mdl_convhull.alpha[:],(1/(selectedStates.Nl*selectedStates.Np))*ones(selectedStates.Nl*selectedStates.Np))

                    elseif selectedStates.version == false && obstacle.obstacle_active == true

                        setvalue(mdl_obstacle.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,4])
                        setvalue(mdl_obstacle.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,4])
                        setvalue(mdl_obstacle.z_Ol[1:mpcParams.N,6],mpcSol.z[2:mpcParams.N+1,1]-posInfo.s_target)
                        setvalue(mdl_obstacle.z_Ol[mpcParams.N+1,6],mpcSol.z[mpcParams.N+1,1]-posInfo.s_target)
                        setvalue(mdl_obstacle.z_Ol[1:mpcParams.N,5],mpcSol.z[2:mpcParams.N+1,2])
                        setvalue(mdl_obstacle.z_Ol[mpcParams.N+1,5],mpcSol.z[mpcParams.N+1,2])
                        setvalue(mdl_obstacle.z_Ol[1:mpcParams.N,4],mpcSol.z[2:mpcParams.N+1,3])
                        setvalue(mdl_obstacle.z_Ol[mpcParams.N+1,4],mpcSol.z[mpcParams.N+1,3])


                        # setvalue(mdl_obstacle.z_Ol[:,2],zCurr[1,2]*ones(mpcParams.N+1))
                        # setvalue(mdl_obstacle.z_Ol[:,3],zCurr[1,3]*ones(mpcParams.N+1))
                        #setvalue(mdl_obstacle.u_Ol,mpcSol.u[1:mpcParams.N,:])
                        setvalue(mdl_obstacle.alpha[:],(1/(selectedStates.Nl*selectedStates.Np))*ones(selectedStates.Nl*selectedStates.Np))

                    end
                elseif lapStatus.currentLap > n_pf+1
                    if selectedStates.version == true

                        setvalue(mdl.z_Ol[:,6],mpcSol.z[:,6]-posInfo.s_target)

                    elseif selectedStates.version == false && obstacle.obstacle_active == false

                        setvalue(mdl_convhull.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,1])
                        setvalue(mdl_convhull.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,1])
                        setvalue(mdl_convhull.z_Ol[1:mpcParams.N,2],mpcSol.z[2:mpcParams.N+1,2])
                        setvalue(mdl_convhull.z_Ol[mpcParams.N+1,2],mpcSol.z[mpcParams.N+1,2])
                        setvalue(mdl_convhull.z_Ol[1:mpcParams.N,3],mpcSol.z[2:mpcParams.N+1,3])
                        setvalue(mdl_convhull.z_Ol[mpcParams.N+1,3],mpcSol.z[mpcParams.N+1,3])
                        setvalue(mdl_convhull.z_Ol[1:mpcParams.N,4],mpcSol.z[2:mpcParams.N+1,4])
                        setvalue(mdl_convhull.z_Ol[mpcParams.N+1,4],mpcSol.z[mpcParams.N+1,4])
                        setvalue(mdl_convhull.z_Ol[1:mpcParams.N,5],mpcSol.z[2:mpcParams.N+1,5])
                        setvalue(mdl_convhull.z_Ol[mpcParams.N+1,5],mpcSol.z[mpcParams.N+1,5])
                        setvalue(mdl_convhull.z_Ol[1:mpcParams.N,6],mpcSol.z[2:mpcParams.N+1,6]-posInfo.s_target)
                        setvalue(mdl_convhull.z_Ol[mpcParams.N+1,6],mpcSol.z[mpcParams.N+1,6]-posInfo.s_target)

                        #setvalue(mdl_convhull.z_Ol[:,6],mpcSol.z[:,6]-posInfo.s_target)
                        setvalue(mdl_convhull.alpha[:],(1/(selectedStates.Nl*selectedStates.Np))*ones(selectedStates.Nl*selectedStates.Np))

                    elseif selectedStates.version == false && obstacle.obstacle_active == true

                        setvalue(mdl_obstacle.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,1])
                        setvalue(mdl_obstacle.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,1])
                        setvalue(mdl_obstacle.z_Ol[1:mpcParams.N,2],mpcSol.z[2:mpcParams.N+1,2])
                        setvalue(mdl_obstacle.z_Ol[mpcParams.N+1,2],mpcSol.z[mpcParams.N+1,2])
                        setvalue(mdl_obstacle.z_Ol[1:mpcParams.N,3],mpcSol.z[2:mpcParams.N+1,3])
                        setvalue(mdl_obstacle.z_Ol[mpcParams.N+1,3],mpcSol.z[mpcParams.N+1,3])
                        setvalue(mdl_obstacle.z_Ol[1:mpcParams.N,4],mpcSol.z[2:mpcParams.N+1,4])
                        setvalue(mdl_obstacle.z_Ol[mpcParams.N+1,4],mpcSol.z[mpcParams.N+1,4])
                        setvalue(mdl_obstacle.z_Ol[1:mpcParams.N,5],mpcSol.z[2:mpcParams.N+1,5])
                        setvalue(mdl_obstacle.z_Ol[mpcParams.N+1,5],mpcSol.z[mpcParams.N+1,5])
                        setvalue(mdl_obstacle.z_Ol[1:mpcParams.N,6],mpcSol.z[2:mpcParams.N+1,6]-posInfo.s_target)
                        setvalue(mdl_obstacle.z_Ol[mpcParams.N+1,6],mpcSol.z[mpcParams.N+1,6]-posInfo.s_target)

                        #setvalue(mdl_obstacle.z_Ol[:,6],mpcSol.z[:,6]-posInfo.s_target)
                        setvalue(mdl_obstacle.alpha[:],(1/(selectedStates.Nl*selectedStates.Np))*ones(selectedStates.Nl*selectedStates.Np))
                    end
                end
            end

            oldSS.oldSS[lapStatus.currentIt,:,lapStatus.currentLap]      = z_est
            oldSS.oldSS_xy[lapStatus.currentIt,:,lapStatus.currentLap]   = x_est


            #  ======================================= Calculate input =======================================
            #println("*** NEW ITERATION # ",i," ***")
            println("Current Lap: ", lapStatus.currentLap, ", It: ", lapStatus.currentIt)#, ", s: ",posInfo.s)
            #println("State Nr. ", i, "    = ", z_est)
            #println("s               = $(posInfo.s)")
            #println("s_total         = $(posInfo.s%posInfo.s_target)")

            mpcParams.Q_obs = ones(selectedStates.Nl*selectedStates.Np)

            if lapStatus.currentLap > 1
                if lapStatus.currentIt == (oldSS.postbuff+2)
                    oldSS.oldSS[oldSS.oldCost[lapStatus.currentLap-1]:oldSS.oldCost[lapStatus.currentLap-1]+oldSS.postbuff+1,1:5,lapStatus.currentLap-1] = zCurr[1:oldSS.postbuff+2,1:5]
                    oldSS.oldSS[oldSS.oldCost[lapStatus.currentLap-1]:oldSS.oldCost[lapStatus.currentLap-1]+oldSS.postbuff+1,6,lapStatus.currentLap-1] = zCurr[1:oldSS.postbuff+2,6] + posInfo.s_target
                end
            end

            ## if obstacles are on the track, find the nearest one

            if obstacle.obstacle_active == true

                obs_temp = obs_curr[lapStatus.currentIt,:,:]

                if posInfo.s_target-posInfo.s < obstacle.obs_detect  # meaning that I could possibly detect obstacles after the finish line

                    index1=find(obs_curr[lapStatus.currentIt,1,:].< obstacle.obs_detect+posInfo.s-posInfo.s_target)  # look for obstacles that could cause problems

                    obs_temp[1,1,index1] = posInfo.s_target + obs_curr[lapStatus.currentIt,1,index1]

                end

                dist,index=findmin(sqrt((obs_temp[1,1,:]-zCurr[lapStatus.currentIt,6]).^2 + (obs_temp[1,2,:]-zCurr[lapStatus.currentIt,5]).^2))

                obs_near = obs_temp[1,:,index]
            end

            # Find coefficients for cost and constraints
            if lapStatus.currentLap > n_pf
                tic()
                coeffConstraintCost(oldTraj,mpcCoeff,posInfo,mpcParams,lapStatus,selectedStates,oldSS,obs_curr[lapStatus.currentIt,:,:],obstacle)
                tt = toq()
                #println("Finished coefficients, t = ",tt," s")
            end

            #println("Starting solving.")

            ### Solve the MPC problem
            tic()
            if lapStatus.currentLap <= n_pf
                z_pf = [zCurr[i,6],zCurr[i,5],zCurr[i,4],norm(zCurr[i,1:2]),acc0]        # use kinematic model and its states
                solveMpcProblem_pathFollow(mdl_pF,mpcSol,mpcParams_pF,trackCoeff,posInfo,modelParams,z_pf,uPrev,lapStatus)
                acc_f[1] = mpcSol.z[1,5]
                acc0 = mpcSol.z[2,5]
            else
                # otherwise: use system-ID-model
                #mpcCoeff.c_Vx[3] = max(mpcCoeff.c_Vx[3],0.1)
                zCurr[i,7] = acc0
                if selectedStates.version == true
                    solveMpcProblem(mdl,mpcSol,mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr[i,:]',uPrev)
                elseif selectedStates.version == false && obstacle.obstacle_active == false
                    solveMpcProblem_convhull(mdl_convhull,mpcSol,mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr[i,:]',uPrev,selectedStates)
                    #solveMpcProblem_test(mdl_test,mpcSol,mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr[i,:]',uPrev,selectedStates)
                elseif selectedStates.version == false && obstacle.obstacle_active == true

                    solveMpcProblem_obstacle(mdl_obstacle,mpcSol,mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr[i,:]',uPrev,selectedStates,obs_near,obstacle)

                end

                acc0 = mpcSol.z[2,7]
                acc_f[1] = mpcSol.z[1,7]
            end
            #if lapStatus.currentLap>n_pf
            #     println("current s= ",posInfo.s)
            #     println("current lap= ",lapStatus.currentLap)
            #     println("selected states= ",selectedStates.selStates)
            #     println("states cost= ",selectedStates.statesCost)
            #     println("old safe set= ",oldSS.oldSS[lapStatus.currentIt:lapStatus.currentIt+10,6,lapStatus.currentLap-1])
            #     println("current control= ", [mpcSol.a_x,mpcSol.d_f])
            #end

            log_t_solv[k+1] = toq()
            #println("time= ",log_t_solv[k+1])

            if obstacle.obstacle_active == true
                obs_curr[lapStatus.currentIt+1,:,:] = obstaclePosition(obs_curr[i,:,:],modelParams,obstacle,posInfo)
            end

            # Send command immediately, only if it is optimal!
            #if mpcSol.solverStatus == :Optimal
            #    opt_count = 0
            #else                        # otherwise use the last optimal input
                #mpcSol.a_x = uPrev[1,1]
                #mpcSol.d_f = uPrev[1,2]
                #opt_count += 1
                # if opt_count >= 5
                #     warn("No optimal solution for $opt_count iterations.")
                # end
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

            selStates_log[:,:,lapStatus.currentIt,lapStatus.currentLap] = selectedStates.selStates  # array to log the selected states in every iteration of every lap
            statesCost_log[:,lapStatus.currentIt,lapStatus.currentLap]  = selectedStates.statesCost # array to log the selected states' costs in every iteration of every lap
            log_cvx[lapStatus.currentIt,:,lapStatus.currentLap]         = mpcCoeff.c_Vx       
            log_cvy[lapStatus.currentIt,:,lapStatus.currentLap]         = mpcCoeff.c_Vy       
            log_cpsi[lapStatus.currentIt,:,lapStatus.currentLap]        = mpcCoeff.c_Psi
            log_input[lapStatus.currentIt,:,lapStatus.currentLap]       = uCurr[i,:] 
            log_mpcCost[lapStatus.currentIt,:,lapStatus.currentLap]     = mpcSol.cost
            log_obs[lapStatus.currentIt,:,:,lapStatus.currentLap]       = obs_curr[i,:,:]
            if lapStatus.currentLap > n_pf
                log_mpcCostSlack[lapStatus.currentIt,:,lapStatus.currentLap]= mpcSol.costSlack
            end

            log_status[lapStatus.currentIt,lapStatus.currentLap]        = mpcSol.solverStatus

            k = k + 1       # counter
            log_sol_status[k]       = mpcSol.solverStatus
            log_state[k,:]          = zCurr[i,:]
            log_cmd[k+1,:]          = uCurr[i,:]                    # the command is going to be pubished in the next iteration
            log_coeff_Cost[:,:,k]   = mpcCoeff.coeffCost      # DONT NEED THIS
            log_coeff_Const[:,:,:,k] = mpcCoeff.coeffConst    # DONT NEED THIS
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

            if lapStatus.currentLap > n_pf
                log_predicted_sol[:,:,lapStatus.currentIt,lapStatus.currentLap] = mpcSol.z
                log_predicted_input[:,:,lapStatus.currentIt,lapStatus.currentLap] = mpcSol.u
                #log_epsalpha[:,lapStatus.currentIt,lapStatus.currentLap]    = mpcSol.eps_alpha
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
                    "t_solv",log_t_solv[1:k],"sol_status",log_sol_status[1:k],"selectedStates",selectedStates,"one_step_error",log_onestep,
                    "oldSS",oldSS,"selStates",selStates_log,"statesCost",statesCost_log,"pred_sol",log_predicted_sol,"pred_input",log_predicted_input,"lapStatus",lapStatus,
                    "posInfo",posInfo,"eps_alpha",log_epsalpha,"cvx",log_cvx,"cvy",log_cvy,"cpsi",log_cpsi,"oldSS_xy",oldSS.oldSS_xy,"input",log_input,
                    "mpcCost",log_mpcCost,"mpcCostSlack",log_mpcCostSlack,"obs_log",log_obs,"final_counter",log_final_counter[1:lapStatus.currentLap])#,"status",log_status)
    println("Exiting LMPC node. Saved data to $log_path.")

    println("number of same s in path following = ",same_sPF)
    println("number of same s in learning MPC = ",same_sLMPC)
    # println("one step prediction error= ",log_onestep[1:20,:,lapStatus.currentLap])
    # println("postions= ",oldSS.oldSS[1:20,6,lapStatus.currentLap])
    # println("selected States= ",selStates_log[:,6,1:20,lapStatus.currentLap] )
    # println("states cost= ",statesCost_log[:,1:20,lapStatus.currentLap])
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
