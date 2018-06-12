#!/usr/bin/env julia
#=
    File name: controllerKin.jl
    Author: Shuqi Xu
    Email: shuqixu@kth.se
    Julia Version: 0.4.7
=#
using RobotOS
@rosimport barc.msg: ECU, pos_info, mpc_visual
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using geometry_msgs.msg
using JuMP
using Ipopt
using JLD

include("library/modules.jl")
include("library/models.jl")
import mpcModels: MdlPf
import solveMpcProblem: solveFd
using Types
import Types: FeatureData
using ControllerHelper, TrackHelper
import DataSavingFuncs: saveFeatureData, featureDataCollect

function main()
    println("Starting controller node.")

    # OBJECTS INITIALIZATION
    BUFFERSIZE  = get_param("BUFFERSIZE")
    raceSet     = RaceSet("Fd")
    track       = Track(createTrack("feature"))
    posInfo     = PosInfo()
    history     = History(BUFFERSIZE,raceSet.num_lap)
    lapStatus   = LapStatus()
    mpcSol      = MpcSol()
    modelParams = ModelParams()
    mpcParams   = MpcParams()

    mpc_vis     = mpc_visual()  # published msg
    cmd         = ECU()         # published msg

    agent       = Agent(track,posInfo,lapStatus,mpcSol,
                        mpcParams,modelParams,raceSet,cmd)

    v_ref       = vcat(1.0,1.0:0.2:3.0)
    Fd          = FeatureData(BUFFERSIZE,length(v_ref))
    
    # UPDATE THE NUMBER OF LAPS TO RACE
    raceSet.num_lap = length(v_ref)

    # OBJECT INITILIZATION AND FUNCTION COMPILING
    mdlFd = MdlPf(agent)
    solveFd(mdlFd,agent,1.0)
    featureDataCollect(agent,Fd)

    # NODE INITIALIZATION
    init_node("controller")
    loop_rate   = Rate(1.0/get_param("controller/dt"))
    ecu_pub     = Publisher("ecu",          ECU,                             queue_size=1)
    vis_pub     = Publisher("mpc_visual",   mpc_visual,                      queue_size=1)
    pos_sub     = Subscriber("pos_info",    pos_info, SE_callback, (agent,), queue_size=1)

    while ! is_shutdown()
        # CONTROL SIGNAL PUBLISHING
        publish(ecu_pub, cmd)

        # THINGS TO DO WHEN LAP SWITCHING
        if lapStatus.nextLap
            # SWITCH LAPS
            lapSwitch(agent,Fd)

            # WARM START WHEN SWITCHING LAPS
            setvalue(mdlFd.z_Ol[:,1],   mpcSol.z_prev[:,1]-track.s)
            
            # SAVE FEATURE DATA AFTER FINISHING ALL LAPS
            if lapStatus.lap > raceSet.num_lap
                saveFeatureData(agent,Fd)
            end
        end

        # CONTROLLER
        solveFd(mdlFd,agent,v_ref[agent.lapStatus.lap])

        # VISUALIZATION UPDATE
        visualUpdate(mpc_vis,agent)
        publish(vis_pub, mpc_vis)
        println("$(agent.mpcSol.sol_status): Lap:",lapStatus.lap,", It:",lapStatus.it," v:$(round(posInfo.v,2))")
        
        # ITERATION UPDATE
        featureDataCollect(agent,Fd)
        rossleep(loop_rate)
    end

    # FEATURE DATA SAVING IF SIMULATION/EXPERIMENT IS KILLED
    saveFeatureData(agent,Fd)
end

if ! isinteractive()
    main()
end
