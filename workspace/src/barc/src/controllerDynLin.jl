#!/usr/bin/env julia
#=
    File name: controllerId.jl
    Author: Shuqi Xu
    Email: shuqixu@berkeley.edu (xushuqi8787@gmail.com)
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
import mpcModels: MdlPf, MdlDynLin
import solveMpcProblem: solvePf, solveDynLin
using Types
using ControllerHelper, TrackHelper
using GPRFuncs, SafeSetFuncs, DataSavingFuncs

function main()
    println("Starting LMPC node.")
    BUFFERSIZE  = get_param("BUFFERSIZE")
    raceSet     = RaceSet("DynLin")
    track       = Track(createTrack(get_param("race_track")))
    posInfo     = PosInfo()
    sysID       = SysID()
    SS          = SafeSet(BUFFERSIZE,raceSet.num_lap)
    history     = History(BUFFERSIZE,raceSet.num_lap)
    lapStatus   = LapStatus()
    mpcSol      = MpcSol()
    mpcParams   = MpcParams()
    modelParams = ModelParams()
    gpData      = GPData("DynLin")

    mpc_vis     = mpc_visual()  # published msg
    cmd         = ECU()         # published msg
    agent       = Agent(track,posInfo,sysID,SS,lapStatus,mpcSol,
                        history,mpcParams,modelParams,gpData,raceSet,cmd)
    
    # OBJECT INITILIZATION AND FUNCTION COMPILING
    mdlPf   = MdlPf(agent)
    solvePf(mdlPf,agent)
    if !raceSet.PF_FLAG
        mdlLMPC = MdlDynLin(agent)
        GPR(agent)
        findSS(agent)
        solveDynLin(mdlLMPC,agent)
    end
    historyCollect(agent)
    gpDataCollect(agent)

    # NODE INITIALIZATION
    init_node("controller")
    loop_rate   = Rate(1.0/get_param("controller/dt"))
    ecu_pub     = Publisher("ecu",          ECU,                             queue_size=1)
    vis_pub     = Publisher("mpc_visual",   mpc_visual,                      queue_size=1)
    pos_sub     = Subscriber("pos_info",    pos_info, SE_callback, (agent,), queue_size=1)

    while ! is_shutdown()
        # CONTROL SIGNAL PUBLISHING
        publish(ecu_pub, cmd)

        # THINGS TO DO DURING LAP SWITCHING
        if lapStatus.nextLap
            # LAP SWITCHING
            lapSwitch(agent)

            # WARM START WHEN SWITCHING THE LAPS
            if raceSet.PF_FLAG
                setvalue(mdlPf.z_Ol[:,1],   mpcSol.z_prev[:,1]-track.s)
            end

            # DATA SAVING AFTER FINISHING ALL LAPS
            if lapStatus.lap > raceSet.num_lap
                saveHistory(agent)
                if !raceSet.GP_LOCAL_FLAG && !raceSet.GP_FULL_FLAG
                    saveGPData(agent)
                end
            end
        end

        # CONTROLLER
        if lapStatus.lap<=1+raceSet.PF_LAP
            solvePf(mdlPf,agent)
        else
            # PATH FOLLOWING DATA SAVING AFTER FINISHING PF LAPS
            if raceSet.PF_FLAG
                savePF(agent)
                println("Finish path following.")
                break # PROCESS CLEANING AFTER SAVING PF DATA
            end

            # GAUSSIAN PROCESS
            GPR(agent)

            # SAFESET CONSTRUCTION
            findSS(agent)

            # SOLVE LMPC
        	solveDynLin(mdlLMPC,agent)

            # COLLECT GAUSSIAN PROCESS FEATURE DATA
            if !raceSet.GP_LOCAL_FLAG && !raceSet.GP_FULL_FLAG && lapStatus.it>1
                gpDataCollect(agent)
            end
        end

        # VISUALIZATION UPDATE
        visualUpdate(mpc_vis,agent)
        publish(vis_pub, mpc_vis)
        println("$(agent.mpcSol.sol_status): Lap:",lapStatus.lap,", It:",lapStatus.it," v:$(round(posInfo.v,2))")
        
        # ITERATION UPDATE
        historyCollect(agent)
        rossleep(loop_rate)
    end

    # DATA SAVING IF SIMULATION/EXPERIMENT IS KILLED
    if !raceSet.PF_FLAG
        saveHistory(agent)
        if !raceSet.GP_LOCAL_FLAG && !raceSet.GP_FULL_FLAG
            saveGPData(agent)
        end
    end
end

if ! isinteractive()
    main()
end
