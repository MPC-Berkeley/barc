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
using JLD

include("helper/classes.jl")
include("helper/coeffConstraintCost.jl")
include("helper/solveMpcProblem.jl")
include("helper/computeCostLap.jl")
include("helper/functions.jl")

function main()
    println("now starting the node")
    # initiate node, set up publisher / subscriber topics
    #init_node("mpc_traj")
    #loop_rate = Rate(10)

    buffersize                  = 700

    # Create data to be saved
    save_oldTraj = zeros(buffersize,4,2,4)  # max. 4 laps

    # Define and initialize variables
    oldTraj                     = OldTrajectory()
    posInfo                     = PosInfo()
    mpcCoeff                    = MpcCoeff()
    lapStatus                   = LapStatus(1,1)
    mpcSol                      = MpcSol()
    trackCoeff                  = TrackCoeff()      # info about track (at current position, approximated)
    modelParams                 = ModelParams()
    mpcParams                   = MpcParams()

    z_est                       = zeros(4)
    coeffCurvature_update       = zeros(5)
    s_start_update              = [0.0]
    #cmd::ECU
    cmd = ECU(0.0,0.0)
    #mdl::JuMP.Model

    #pub                         = Publisher("ecu", ECU, queue_size=10)
    #pub2                        = Publisher("logging", Logging, queue_size=10)
    # The subscriber passes 3 arguments (s_start, coeffCurvature and z_est) which are updated by the callback function:
    #s1                          = Subscriber("pos_info", pos_info, SE_callback, (s_start_update,coeffCurvature_update,z_est,),queue_size=10)

    InitializeParameters(mpcParams,trackCoeff,modelParams,posInfo,oldTraj,mpcCoeff,lapStatus,buffersize)
    println("Finished initialization.")
    # Lap parameters
    switchLap               = false     # initialize lap lap trigger
    s_lapTrigger            = 0.3       # next lap is triggered in the interval s_start in [0,s_lapTrigger]
    
    # buffer in current lap
    zCurr       = zeros(10000,4)    # contains state information in current Lap (max. 10'000 steps)
    uCurr       = zeros(10000,2)    # contains input information

    zCurr_export = zeros(buffersize,4)
    uCurr_export = zeros(buffersize,2)

    # DEFINE MODEL ***************************************************************************
    # ****************************************************************************************
    println("Building model...")

    z_Init          = zeros(4)

    mdl             = MpcModel()
    InitializeModel(mdl,mpcParams,modelParams,trackCoeff,z_Init)
    

    # Initial solve:
    println("Initial solve...")
    solve(mdl.mdl)
    solve(mdl.mdl)
    println("Finished.")

    z_sim = zeros(4)
    for j=1:100
        z_sim[1] += 0.05
        z_est = z_sim
        if z_est[1] > 0         # check if data has been received (s > 0) 
            println("Received data: z_est = $z_est")
            println("curvature = $coeffCurvature_update")
            println("s_start = $s_start_update")           

            # ============================= Initialize iteration parameters =============================
            lapStatus.currentIt += 1                            # count iteration

            i                   = lapStatus.currentIt           # current iteration number, just to make notation shorter
            zCurr[i,:]          = z_est                         # update state information
            posInfo.s           = z_est[1]                      # update position info
            posInfo.s_start     = s_start_update[1]
            trackCoeff.coeffCurvature = coeffCurvature_update


            # ======================================= Lap trigger =======================================
            # This part takes pretty long (about 0.6 seconds on my Mac) and should be faster!
            if (posInfo.s_start + posInfo.s)%posInfo.s_target <= s_lapTrigger && switchLap      # if we are switching to the next lap...
                # ... then select and save data
                z_sim[1] = 0
                println("Saving data")
                tic()
                saveOldTraj(oldTraj,zCurr,uCurr,lapStatus,buffersize,dt)
                save_oldTraj[:,:,:,lapStatus.currentLap] = oldTraj.oldTraj[:,:,:]
                zCurr[1,:] = zCurr[i,:]         # reset counter to 1 and set current state
                uCurr[1,:] = uCurr[i+1,:]       # ... and input
                i                     = 1       
                lapStatus.currentLap += 1       # start next lap
                lapStatus.currentIt   = 1       # reset current iteration
                switchLap = false

                tt = toq()
                println("Saved data, t = $tt")
                println("======================================== NEXT LAP ========================================")
                println("cost: $(oldTraj.oldCost)")
                println("oldTraj.oldTraj[:,1,1]:")
                println(oldTraj.oldTraj[:,1,1])
                println("oldTraj.oldTraj[:,1,2]:")
                println(oldTraj.oldTraj[:,1,2])
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
            if lapStatus.currentLap > 1
                coeffConstraintCost(oldTraj,lapStatus,mpcCoeff,posInfo,mpcParams)
            end
            tt = toq()
            println("Finished coefficients, t = $tt s")
            #println("Found coefficients: mpcCoeff = $mpcCoeff")
            # Solve the MPC problem
            tic()
            solveMpcProblem(mdl,mpcSol,mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr[i,:]',uCurr[i,:]')
            tt = toq()
            # Write in current input information
            uCurr[i+1,:]  = [mpcSol.a_x mpcSol.d_f]
            #println("trackCoeff = $trackCoeff")
            println("Finished solving, status: $(mpcSol.solverStatus), u = $(uCurr[i,:]), t = $tt s")
            # ... and publish data
            #cmd = ECU(mpcSol.a_x, mpcSol.d_f)
            #publish(pub, cmd)

            zCurr[i,1] = (posInfo.s_start + posInfo.s)%posInfo.s_target   # save absolute position in s (for oldTrajectory)
            println("\n")
        
        else
            println("No estimation data received!")
        end
        #rossleep(loop_rate)
    end
    # Save simulation data to file

    log_path = "$(homedir())/simulations/output_LMPC.jld"
    save(log_path,"oldTraj",save_oldTraj)
    println("Exiting LMPC node. Saved data.")

end

if ! isinteractive()
    main()
end
