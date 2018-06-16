#!/usr/bin/env julia
#=
    File name: controllerId.jl
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

include("Library/modules.jl")
include("Library/models.jl")
import mpcModels: MdlPf, MdlKinLin
import solveMpcProblem: solvePf, solveKinLin
using Types
using ControllerHelper, TrackHelper
using SysIDFuncs, GPRFuncs, SafeSetFuncs, DataSavingFuncs

function main()
    println("Starting LMPC node.")
    BUFFERSIZE  = get_param("BUFFERSIZE")

    if get_param("controller/TV_FLAG")
        raceSet = RaceSet("KinLin_TV")
    else
        raceSet = RaceSet("KinLin_TI")
    end

    # OBJECTS INITIALIZATION
    track       = Track(createTrack(get_param("race_track")))
    track_Fd    = Track(createTrack("feature"))
    posInfo     = PosInfo()
    sysID       = SysID()
    SS          = SafeSet(BUFFERSIZE,raceSet.num_lap)
    history     = History(BUFFERSIZE,raceSet.num_lap)
    lapStatus   = LapStatus()
    mpcSol      = MpcSol()
    modelParams = ModelParams()
    mpcParams   = MpcParams()

    if get_param("controller/TV_FLAG")
        gpData = GPData("KinLin_TV")
    else
        gpData = GPData("KinLin_TI")
    end

    mpc_vis     = mpc_visual()  # published msg
    cmd         = ECU()         # published msg
    agent       = Agent(track,posInfo,sysID,SS,lapStatus,mpcSol,
                        history,mpcParams,modelParams,gpData,raceSet,cmd)
    
    # OBJECT INITILIZATION AND FUNCTION COMPILING
    mdlPf   = MdlPf(agent)
    solvePf(mdlPf,agent)
    if !raceSet.PF_FLAG
        mdlLMPC = MdlKinLin(agent)
        gprDyn(agent)
        findSS(agent)
        solveKinLin(mdlLMPC,agent)

        # DIFFERENT OPTIONS FOR SELECTING FEATURE DATA FOR SYS ID
        buildFeatureSetFromHistory(agent)
        # data = load("$(homedir())/$(raceSet.folder_name)/FD.jld")
        # featureData = data["featureData"]
        # buildFeatureSetFromDataSet(agent,featureData)
        # buildFeatureSetFromBoth(agent,featureData)
    end
    historyCollect(agent)
    gpResultCollect(agent)
    gpErrorCollect(agent)
    gpFeatureCollect(agent)

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
            else
                # DIFFERENT OPTIONS FOR SELECTING FEATURE DATA FOR SYS ID
                buildFeatureSetFromHistory(agent)
                # buildFeatureSetFromDataSet(agent,featureData)
                # buildFeatureSetFromBoth(agent,featureData)
            end

            # DATA SAVING AFTER FINISHING ALL LAPS
            if lapStatus.lap > raceSet.num_lap
                saveHistory(agent)
                if raceSet.GP_LOCAL_FLAG || raceSet.GP_FULL_FLAG
                    saveGpResultData(agent)
                else
                    saveGpFeatureData(agent)
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
                break
            end
            # GAUSSIAN PROCESS
            if raceSet.GP_LOCAL_FLAG || raceSet.GP_FULL_FLAG
                gprDyn(agent)
                gpResultCollect(agent)
                findSS(agent)
                solveKinLin(mdlLMPC,agent)
                gpErrorCollect(agent)
            else
                findSS(agent)
                solveKinLin(mdlLMPC,agent)
                gpFeatureCollect(agent)
            end
        end

        # VISUALIZATION UPDATE
        visualUpdate(mpc_vis,agent)
        publish(vis_pub, mpc_vis)
        # println("$(agent.mpcSol.sol_status): Lap:",lapStatus.lap,", It:",lapStatus.it," v:$(round(posInfo.v,2))")
        
        # ITERATION UPDATE
        historyCollect(agent)
        rossleep(loop_rate)
    end

    # DATA SAVING IF SIMULATION/EXPERIMENT IS KILLED
    if !raceSet.PF_FLAG
        saveHistory(agent)
        if raceSet.GP_LOCAL_FLAG || raceSet.GP_FULL_FLAG
            saveGpResultData(agent)
        else
            saveGpFeatureData(agent)
        end
    end
end

if ! isinteractive()
    main()
end
