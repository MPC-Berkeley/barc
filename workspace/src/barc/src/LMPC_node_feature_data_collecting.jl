#!/usr/bin/env julia

using RobotOS
@rosimport barc.msg: ECU, pos_info, mpc_solution
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using geometry_msgs.msg
using JuMP
using Ipopt
using JLD

# log msg
include("barc_lib/classes.jl")
include("barc_lib/LMPC/MPC_models.jl")
include("barc_lib/LMPC/functions.jl")
include("barc_lib/LMPC/coeffConstraintCost.jl")
include("barc_lib/LMPC/solveMpcProblem.jl")
include("barc_lib/simModel.jl")

# This function is called whenever a new state estimate is received.
# It saves this estimate in oldTraj and uses it in the MPC formulation (see in main)
function SE_callback(msg::pos_info,acc_f::Array{Float64},lapStatus::LapStatus,posInfo::PosInfo,mpcSol::MpcSol,oldTraj::OldTrajectory,z_est::Array{Float64,1},x_est::Array{Float64,1})         # update current position and track data
    # update mpc initial condition
    z_est[:]                  = [msg.v_x,msg.v_y,msg.psiDot,msg.epsi,msg.ey,msg.s,acc_f[1]] # the last variable is filtered acceleration
    x_est[:]                  = [msg.x,msg.y,msg.psi,msg.v]

    # check if lap needs to be switched
    if z_est[6] <= lapStatus.s_lapTrigger && lapStatus.switchLap
        lapStatus.currentLap += 1
        lapStatus.currentIt = 1
        lapStatus.nextLap = true
        lapStatus.switchLap = false
    elseif z_est[6] > lapStatus.s_lapTrigger
        lapStatus.switchLap = true
    end

    # save current state in oldTraj
    oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap],:,lapStatus.currentLap] = z_est
    oldTraj.count[lapStatus.currentLap] += 1
end

# This is the main function, it is called when the node is started.
function main()
    println("Starting LMPC_Feature_Collecting node.")
    const BUFFERSIZE       = 500
    const LMPC_LAP         = 10
    const PF_FLAG          = true   # true:only pF,     false:1 warm-up lap and LMPC
    const LMPC_FLAG        = true   # true:IDEN_MODEL,  false:IDEN_KIN_LIN_MODEL
    const FEATURE_FLAG     = false  # true:8-shape,     false:history (this requires the corresponding change in 3 palces)
    const TI_TV_FLAG       = true   # true:TI,          false:TV
    const GP_LOCAL_FLAG    = false  # true:local GPR
    const GP_FULL_FLAG     = false  # true:full GPR
    const N                = 16
    const delay_df         = 3
    const delay_a          = 1
    track_data       = createTrack("feature")
    track            = Track(track_data)
    oldTraj          = OldTrajectory()
    posInfo          = PosInfo();  posInfo.s_target=track.s;
    lapStatus        = LapStatus(1,1,false,false,0.3)
    mpcCoeff         = MpcCoeff()
    mpcCoeff_dummy   = MpcCoeff()
    mpcSol           = MpcSol()
    modelParams      = ModelParams()
    mpcParams_pF     = MpcParams()
    mpcParams        = MpcParams()
    mpcParams_4s     = MpcParams()
    selectedStates   = SelectedStates()
    oldSS            = SafeSetData()
    z_est            = zeros(7)          # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
    x_est            = zeros(4)          # (x, y, psi, v)
    cmd              = ECU()             # CONTROL SIGNAL MESSAGE INITIALIZATION
    mpcSol_to_pub    = mpc_solution()    # MPC SOLUTION PUBLISHING MESSAGE INITIALIZATION
    InitializeParameters(mpcParams,mpcParams_4s,mpcParams_pF,modelParams,mpcSol,
                         selectedStates,oldSS,oldTraj,mpcCoeff,mpcCoeff_dummy,
                         LMPC_LAP,delay_df,delay_a,N,BUFFERSIZE)
    
    # FEATURE DATA INITIALIZATION
    v_ref = vcat([0.8],0.8:0.05:2.5)

    feature_z   = zeros(100000,6,2)
    feature_u   = zeros(100000,2)

    # DATA LOGGING VARIABLE INITIALIZATION
    k = 1 # counter initialization, k is the counter from the beginning of the experiment

    # MODEL INITIALIZATION
    mdl_pF           = MpcModel_pF(mpcParams_pF,modelParams)
    
    # NODE INITIALIZATION
    init_node("mpc_traj")
    loop_rate = Rate(1/modelParams.dt)
    pub = Publisher("ecu", ECU, queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
    mpcSol_pub = Publisher("mpc_solution", mpc_solution, queue_size=1)::RobotOS.Publisher{barc.msg.mpc_solution}
    # The subscriber passes arguments (coeffCurvature and z_est) which are updated by the callback function:
    acc_f = [0.0]
    s1 = Subscriber("pos_info", pos_info, SE_callback, (acc_f,lapStatus,posInfo,mpcSol,oldTraj,z_est,x_est),queue_size=50)::RobotOS.Subscriber{barc.msg.pos_info}
    # Note: Choose queue size long enough so that no pos_info packets get lost! They are important for system ID!
    run_id = get_param("run_id")
    println("Finished initialization.")
    
    mpcSol.df_his = zeros(mpcParams.delay_df)
    mpcSol.z = zeros(mpcParams_pF.N+1,4)
    mpcSol.u = zeros(mpcParams_pF.N,2)
    for i in 2:mpcParams_pF.N+1
        mpcSol.z[i,:]=car_sim_kin(mpcSol.z[i-1,:],mpcSol.u[i-1,:],track,modelParams)
    end
    mpcSol.a_x = 0
    mpcSol.d_f = 0
    z_prev = mpcSol.z
    u_prev = mpcSol.u

    while ! is_shutdown()
        if z_est[6] > 0    
            # CONTROL SIGNAL PUBLISHING
            cmd.header.stamp = get_rostime()
            mpcSol_to_pub.header.stamp = get_rostime()     
            publish(pub, cmd)
            publish(mpcSol_pub, mpcSol_to_pub)

            # TRACK FEATURE DATA COLLECTING: it is important to put this at the beginning of the iteration to make the data consistant
            if lapStatus.currentLap>1
                k = k + 1 # start counting from the second lap.
                
                # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
                feature_z[k,:,1] = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
                feature_u[k,1] = u_sol[2,1] # acceleration delay is from MPC itself
                feature_u[k+mpcParams.delay_df-1,2] = u_sol[1+mpcParams.delay_df,2] # another 2 steps delay is from the system
                # it should be the same to store the u_sol[2] to the current position, k, since it is fixed in the optimization problem.
                feature_z[k-1,:,2] = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
                # println("feature_u from MPC",feature_u[k,:])
                # println("current publishing u",u_sol[2,:])
            end


            # LAP SWITCHING
            if lapStatus.nextLap
                println("Finishing one lap at iteration ",lapStatus.currentIt)
                lapStatus.nextLap = false
                if mpcSol.z[1,1]>posInfo.s_target
                    # WARM START SWITCHING
                    setvalue(mdl_pF.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,1]-posInfo.s_target)
                    setvalue(mdl_pF.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,1]-posInfo.s_target)
                end
            end

            # OPTIMIZATION
            println("Current Lap: ", lapStatus.currentLap, ", It: ", lapStatus.currentIt)

            # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
            z_curr = [z_est[6],z_est[5],z_est[4],sqrt(z_est[1]^2+z_est[2]^2)]
 
            (z_sol,u_sol,sol_status)=solveMpcProblem_featureData(mdl_pF,mpcParams_pF,modelParams,mpcSol,z_curr,z_prev,u_prev,track,v_ref[lapStatus.currentLap])
            mpcSol.z = z_sol
            mpcSol.u = u_sol

            # # ADDITIONAL NOISE ADDING, WHICH IS ONLY FOR SIMULATION
            # n=randn(2)
            # n_thre = [0.01,0.00001]
            # n.*=n_thre
            # n=min(n,n_thre)
            # n=max(n,[0,-0.00001])
            # u_sol[1+mpcParams_pF.delay_a,1]+=n[1]
            # # u_sol[1+mpcParams_pF.delay_df,2]+=n[2]
            
            mpcSol.a_x = u_sol[1+mpcParams_pF.delay_a,1] 
            mpcSol.d_f = u_sol[1+mpcParams_pF.delay_df,2]
            if length(mpcSol.df_his)==1
                mpcSol.df_his[1] = u_sol[1+mpcParams_pF.delay_df,2]
            else
                # INPUT DELAY HISTORY UPDATE
                mpcSol.df_his[1:end-1] = mpcSol.df_his[2:end]
                mpcSol.df_his[end] = u_sol[1+mpcParams_pF.delay_df,2]
            end

            z_prev = z_sol
            u_prev = u_sol
            println("Feature collecting solver status is = $sol_status")

            cmd.motor = convert(Float32,mpcSol.a_x)
            cmd.servo = convert(Float32,mpcSol.d_f)

            # VISUALIZATION COORDINATE CALCULATION FOR view_trajectory.jl NODE
            (z_x,z_y) = trackFrame_to_xyFrame(z_sol,track)
            mpcSol_to_pub.z_x = z_x
            mpcSol_to_pub.z_y = z_y

            # TRACK FEATURE DATA COLLECTING
            if lapStatus.currentLap==length(v_ref) && z_est[6] > track.s[end]-0.5
                log_path = "$(homedir())/simulations/Feature_Data/FeatureDataCollecting-$(run_id[1:4]).jld"
                if isfile(log_path)
                    log_path = "$(homedir())/simulations/Feature_Data/FeatureDataCollecting-$(run_id[1:4])-2.jld"
                    warn("Warning: File already exists.")
                end
                # CUT THE FRONT AND REAR TAIL BEFORE SAVING THE DATA
                feature_z = feature_z[2+mpcParams.delay_df-1:k-1,:,:]
                feature_u = feature_u[2+mpcParams.delay_df-1:k-1,:]
                # DATA SAVING
                save(log_path,"feature_z",feature_z,"feature_u",feature_u)
            end

            lapStatus.currentIt += 1
        else
            println("No estimation data received!")
        end
        rossleep(loop_rate)
    end # END OF THE WHILE LOOP
    # DATA SAVING, trying to save as much as feature data as possible to do sys_id
    log_path = "$(homedir())/simulations/Feature_Data/FeatureDataCollecting-$(run_id[1:4]).jld"
    if isfile(log_path)
        log_path = "$(homedir())/simulations/Feature_Data/FeatureDataCollecting-$(run_id[1:4])-2.jld"
        warn("Warning: File already exists.")
    end
    # CUT THE FRONT AND REAR TAIL BEFORE SAVING THE DATA
    feature_z = feature_z[2+mpcParams.delay_df-1:k-1,:,:]
    feature_u = feature_u[2+mpcParams.delay_df-1:k-1,:]
    # DATA SAVING
    save(log_path,"feature_z",feature_z,"feature_u",feature_u,"oldTraj",oldTraj)
end


if ! isinteractive()
    main()
end

# zCurr[1] = v_x
# zCurr[2] = v_y
# zCurr[3] = psiDot
# zCurr[4] = ePsi
# zCurr[5] = eY
# zCurr[6] = s
