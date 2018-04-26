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
include("barc_lib/LMPC/functions.jl")
include("barc_lib/LMPC/MPC_models.jl")
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
        oldTraj.idx_end[lapStatus.currentLap] = oldTraj.count[lapStatus.currentLap]
        oldTraj.oldCost[lapStatus.currentLap] = oldTraj.idx_end[lapStatus.currentLap] - oldTraj.idx_start[lapStatus.currentLap]
        lapStatus.currentLap += 1
        lapStatus.currentIt = 1
        lapStatus.nextLap = true
        lapStatus.switchLap = false
    elseif z_est[6] > lapStatus.s_lapTrigger
        lapStatus.switchLap = true
    end

    # save current state in oldTraj
    oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap],:,lapStatus.currentLap] = z_est
    oldTraj.oldInput[oldTraj.count[lapStatus.currentLap],:,lapStatus.currentLap] = [msg.u_a,msg.u_df]
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
    println("Starting LMPC_Feature_Collecting node.")

    # PARAMETER INITILIZATION
    buffersize       = 5000       # size of oldTraj buffers

    v = 2.5
    max_a=7.6;
    R=v^2/max_a
    max_c=1/R
    angle=(pi+pi/2)-0.105
    R_kin = 0.8
    num_kin = Int(round(angle/ ( 0.03/R_kin ) * 2))
    num = max(Int(round(angle/ ( 0.03/R ) * 2)),num_kin)
    # num*=2
    track_data=[num -angle;
                num  angle]
    track            = Track(track_data)
    oldTraj          = OldTrajectory()
    posInfo          = PosInfo();  posInfo.s_target=track.s;
    mpcCoeff         = MpcCoeff()
    lapStatus        = LapStatus(1,1,false,false,0.3)
    mpcSol           = MpcSol()
    modelParams      = ModelParams()
    mpcParams_pF     = MpcParams()
    mpcParams        = MpcParams()  # a dummy parameter in feature data collecting
    selectedStates   = SelectedStates()
    oldSS            = SafeSetData()
    z_est            = zeros(7)          # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
    x_est            = zeros(4)          # (x, y, psi, v)
    cmd              = ECU()             # CONTROL SIGNAL MESSAGE INITIALIZATION
    mpcSol_to_pub    = mpc_solution()    # MPC SOLUTION PUBLISHING MESSAGE INITIALIZATION

    # FEATURE DATA INITIALIZATION
    # 100000 IS THE BUFFER FOR FEATURE DATA
    # v_ref_dummy = [i for i in 0.6:0.1:2.5]
    v_ref = vcat([1.5],1.5:0.05:2.5)
    # v_ref = [1]
    # for v in v_ref_dummy
    #     v_ref = vcat(v_ref,[v,v])
    # end

    num_lap = length(v_ref)
    feature_z = zeros(100000,6,2)
    feature_u = zeros(100000,2)

    # DATA LOGGING VARIABLE INITIALIZATION
    # selStates_log    = zeros(selectedStates.Nl*selectedStates.Np,6,buffersize,30)
    # statesCost_log   = zeros(selectedStates.Nl*selectedStates.Np,buffersize,30)
    log_cvx                     = zeros(buffersize,3,num_lap)
    log_cvy                     = zeros(buffersize,4,num_lap)
    log_cpsi                    = zeros(buffersize,3,num_lap)
    log_status                  = Array(Symbol,buffersize,num_lap)
    k = 1 # counter initialization, k is the counter from the beginning of the experiment

    InitializeParameters(mpcParams,mpcParams_pF,modelParams,posInfo,oldTraj,mpcCoeff,lapStatus,buffersize,selectedStates,oldSS)

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

    # posInfo.s_target        = 19.11 #17.91 #19.14#17.94#17.76#24.0
    
    mpcSol.z = zeros(11,4)
    mpcSol.u = zeros(10,2)
    for i in 2:11
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
                feature_u[k,:] = u_sol[2,:]
                feature_z[k-1,:,2] = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
                println("Vy: $(z_est[2]) psi_dot: $(z_est[3])")
                # So bsides the zeros tail, the first and last points will be removed.
                # if lapStatus.currentLap==length(v_ref) && z_est[6] > track.s[end]-0.5
                #     log_path = "$(homedir())/simulations/Feature_Data/FeatureDataCollecting.jld"
                #     if isfile(log_path)
                #         log_path = "$(homedir())/simulations/Feature_Data/FeatureDataCollecting-2.jld"
                #         warn("Warning: File already exists.")
                #     end
                #     # CUT THE FRONT AND REAR TAIL BEFORE SAVING THE DATA
                #     feature_z = feature_z[2:k-1,:,:]
                #     feature_u = feature_u[2:k-1,:]
                #     # DATA SAVING
                #     save(log_path,"feature_z",feature_z,"feature_u",feature_u)
                # end
            end

            # LAP SWITCHING
            if lapStatus.nextLap
                println("Finishing one lap at iteration ",lapStatus.currentIt)
                # SAFE SET COST UPDATE
                # log_final_counter[lapStatus.currentLap-1] = k
                oldSS.oldCost[lapStatus.currentLap-1] = lapStatus.currentIt
                cost2target                           = zeros(buffersize) # array containing the cost to arrive from each point of the old trajectory to the target            
                for j = 1:buffersize
                    cost2target[j] = (lapStatus.currentIt-j+1)  
                end
                oldSS.cost2target[:,lapStatus.currentLap-1] = cost2target
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
            # println("s: ", z_curr[1], "x: ", x_est[1], "y: ", x_est[2])
            # println("ey: ",z_est[5])

            (z_sol,u_sol,sol_status)=solveMpcProblem_featureData(mdl_pF,mpcParams_pF,modelParams,z_curr,z_prev,u_prev,track,v_ref[lapStatus.currentLap])
            mpcSol.z = z_sol
            mpcSol.u = u_sol
            # if rand() > 0.3
            #     mpcSol.a_x = u_sol[2,1]
            # else
                # u_sol[2,1] = -rand()
                mpcSol.a_x = u_sol[2,1]
            # end
            mpcSol.d_f = u_sol[2,2]
            z_prev = z_sol
            u_prev = u_sol
            println("Feature collecting solver status is = $sol_status")

            cmd.motor = convert(Float32,mpcSol.a_x)
            cmd.servo = convert(Float32,mpcSol.d_f)

            # VISUALIZATION COORDINATE CALCULATION FOR view_trajectory.jl NODE
            (z_x,z_y) = trackFrame_to_xyFrame(z_sol,track)
            mpcSol_to_pub.z_x = z_x
            mpcSol_to_pub.z_y = z_y

            # # DATA WRITING AND COUNTER UPDATE
            # log_cvx[lapStatus.currentIt,:,lapStatus.currentLap]         = mpcCoeff.c_Vx       
            # log_cvy[lapStatus.currentIt,:,lapStatus.currentLap]         = mpcCoeff.c_Vy       
            # log_cpsi[lapStatus.currentIt,:,lapStatus.currentLap]        = mpcCoeff.c_Psi
            # log_status[lapStatus.currentIt,lapStatus.currentLap]        = mpcSol.solverStatus

            # TRACK FEATURE DATA COLLECTING
            # if lapStatus.currentLap>1
            #     k = k + 1 # start counting from the second lap.
            #     # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
            #     feature_z[k,:,1] = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
            #     feature_u[k,:] = u_sol[2,:]
            #     feature_z[k-1,:,2] = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
            #     # So bsides the zeros tail, the first and last points will be removed.
            if lapStatus.currentLap==length(v_ref) && z_est[6] > track.s[end]-0.5
                log_path = "$(homedir())/simulations/Feature_Data/FeatureDataCollecting.jld"
                if isfile(log_path)
                    log_path = "$(homedir())/simulations/Feature_Data/FeatureDataCollecting-2.jld"
                    warn("Warning: File already exists.")
                end
                # CUT THE FRONT AND REAR TAIL BEFORE SAVING THE DATA
                feature_z = feature_z[2:k-1,:,:]
                feature_u = feature_u[2:k-1,:]
                # DATA SAVING
                save(log_path,"feature_z",feature_z,"feature_u",feature_u)
            end
            # end

            lapStatus.currentIt += 1
        else
            println("No estimation data received!")
        end
        rossleep(loop_rate)
    end # END OF THE WHILE LOOP
    # DATA SAVING
    # log_path = "$(homedir())/simulations/Feature_Data/FeatureDataCollecting-$(run_id[1:4]).jld"
    # if isfile(log_path)
    #     log_path = "$(homedir())/simulations/Feature_Data/FeatureDataCollecting-$(run_id[1:4])-2.jld"
    #     warn("Warning: File already exists.")
    # end
    # save(log_path,"feature_z",feature_z,"feature_u",feature_u)
    # println("Exiting LMPC node. Saved data to $log_path.")
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
