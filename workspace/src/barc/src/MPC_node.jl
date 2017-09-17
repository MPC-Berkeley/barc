#!/usr/bin/env julia

using RobotOS
@rosimport barc.msg: ECU, pos_info
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using geometry_msgs.msg
using JuMP
using Ipopt

include("barc_lib/classes.jl")
include("barc_lib/MPC/functions.jl")
include("barc_lib/MPC/MPC_models.jl")
include("barc_lib/MPC/solveMpcProblem.jl")
include("barc_lib/simModel.jl")

#using PyCall
#unshift!(PyVector(pyimport("sys")["path"]), "")
#@pyimport Localization_helpers.Localization as Loc
#L = Loc.Localization()["create_track"]()

# zCurr[1] = v_x
# zCurr[2] = v_y
# zCurr[3] = psiDot
# zCurr[4] = ePsi
# zCurr[5] = eY
# zCurr[6] = s

# This function is called whenever a new state estimate is received.
function SE_callback(msg::pos_info,acc_f::Array{Float64},lapStatus::LapStatus,posInfo::PosInfo,mpcSol::MpcSol,trackCoeff::TrackCoeff,z_est::Array{Float64,1})         # update current position and track data
    # update mpc initial condition
    z_est[:]                  = [msg.v_x,msg.v_y,msg.psiDot,msg.epsi,msg.ey,msg.s,acc_f[1]]             # use z_est as pointer
    trackCoeff.coeffCurvature = msg.coeffCurvature

    # check if lap needs to be switched
    if z_est[6] <= lapStatus.s_lapTrigger && lapStatus.switchLap
        lapStatus.currentLap += 1
        lapStatus.nextLap = true
        lapStatus.switchLap = false
    elseif z_est[6] > lapStatus.s_lapTrigger
        lapStatus.switchLap = true
    end
end

# This is the main function, it is called when the node is started.
function main()
    println("Starting MPC node ...............")

    # Define and initialize variables
    # ---------------------------------------------------------------
    # General MPC variables
    posInfo                     = PosInfo()
    mpcCoeff                    = MpcCoeff()
    lapStatus                   = LapStatus(1,1,false,false,0.3)
    mpcSol                      = MpcSol()
    trackCoeff                  = TrackCoeff()      # info about track (at current position, approximated)
    mdlParams                   = ModelParams()
    mpcParams                   = MpcParams()

    InitializeParameters(mpcParams,trackCoeff,mdlParams,posInfo,lapStatus)
    mdl = MpcModel(mpcParams,mdlParams,trackCoeff)

    # ROS-specific variables
    z_est                       = zeros(7)          # this is a buffer that saves current state information (xDot, yDot, psiDot, ePsi, eY, s)
    cmd                         = ECU()             # command type
    coeffCurvature_update       = zeros(trackCoeff.nPolyCurvature+1)
    
    acc_f = [0.0]

    # Initialize ROS node and topics
    init_node("mpc_traj")
    loop_rate = Rate(1/mdlParams.dt)
    pub = Publisher("ecu", ECU, queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
    s1 = Subscriber("pos_info", pos_info, SE_callback, (acc_f,lapStatus,posInfo,mpcSol,trackCoeff,z_est),queue_size=50)::RobotOS.Subscriber{barc.msg.pos_info}

    println("Finished initialization.")
    

    # Specific initializations:
    lapStatus.currentLap    = 1
    lapStatus.currentIt     = 1
    posInfo.s_target        = 7.24     # distance of track
    
    mpcSol.z = zeros(11,4)
    mpcSol.u = zeros(10,2) 

    uPrev = zeros(10,2)     # saves the last 10 inputs (1 being the most recent one)

    acc0 = 0.0
    u_opt = [0.0 0.0]

    # Start node
    while ! is_shutdown()
        if z_est[6] > 0         # check if data has been received (s > 0)

            # ============================= UPDATE STATE ESTIMATE =============================
            n_mpc                       = lapStatus.currentIt           # current iteration number, just to make notation shorter
            zCurr                       = copy(z_est)                   # update state information
            posInfo.s                   = z_est[6]                    # update position info

            # ======================================= DISPLAY PROGRESS =======================================
            if lapStatus.nextLap                
                println("Finishing one lap at iteration ", n_mpc)
                n_mpc                   = 1
                lapStatus.currentIt     = 1       
                lapStatus.nextLap       = false
                setvalue(mdl.z_Ol[:,1], mpcSol.z[:,1]-posInfo.s_target)
            end
            #println("Current Lap: ", lapStatus.currentLap, ", It: ", lapStatus.currentIt)
           
            #  ======================================= COMPUTE CONTROL =======================================
            z_pf = [zCurr[6],zCurr[5],zCurr[4],norm(zCurr[1:2]),acc0]        # use kinematic model and its states
            solveMpcProblem(mdl,mpcSol,mpcParams,trackCoeff,posInfo,mdlParams,z_pf,uPrev)
            acc_f[1] = mpcSol.z[1,5]
            acc0 = mpcSol.z[2,5]
            cmd.motor = convert(Float32,mpcSol.a_x)
            cmd.servo = convert(Float32,mpcSol.d_f)

            # Write current input information
            u_opt = [mpcSol.a_x mpcSol.d_f]
            uPrev = circshift(uPrev,1)
            uPrev[1,:] = u_opt
            lapStatus.currentIt += 1

            # ============================= PUBLISH COMMANDS =============================
            cmd.header.stamp = get_rostime()
            publish(pub, cmd)
            println("steering angle: ", u_opt[2])
        else
            println("No estimation data received!")
        end
        rossleep(loop_rate)
    end

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
