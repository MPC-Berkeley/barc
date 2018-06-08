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

include("Library/modules.jl")
include("Library/models.jl")
import mpcModels: MdlPf, MdlKin
import solveMpcProblem: solvePf, solveKin
using Types
using ControllerHelper, TrackHelper
using GPRFuncs, SafeSetFuncs, DataSavingFuncs

function main()
    println("Starting LMPC node.")
    BUFFERSIZE  = get_param("BUFFERSIZE")
    raceSet     = RaceSet("KIN")
    track       = Track(createTrack("basic"))
    posInfo     = PosInfo()
    sysID       = SysID()
    SS          = SafeSet(BUFFERSIZE,raceSet.num_lap)
    history     = History(BUFFERSIZE,raceSet.num_lap)
    lapStatus   = LapStatus()
    mpcSol      = MpcSol()
    mpcParams   = MpcParams()
    gpData      = GPData("KIN")
    mpc_vis     = mpc_visual()  # published msg
    cmd         = ECU()         # published msg
    agent       = Agent(track,posInfo,sysID,SS,lapStatus,mpcSol,
                        history,mpcParams,gpData,raceSet,cmd)
    
    mdlPf   = MdlPf(agent)
    solvePf(mdlPf,agent)
    if !raceSet.PF_FLAG
        mdlLMPC = MdlKin(agent)
        GPR(agent)
        findSS(agent)
        solveKin(mdlLMPC,agent)
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
        if posInfo.s > 0
            # LAP SWITCHING
            if lapStatus.nextLap
                lapSwitch(agent)
                # if raceSet.PF_FLAG
                #     setvalue(mdlPf.z_Ol[:,1],   mpcSol.z_prev[:,1]-track.s)
                # else
                #     setvalue(mdlLMPC.z_Ol[:,1], mpcSol.z_prev[:,1]-track.s)
                #     println(mpcSol.z_prev[:,1]-track.s)
                # end
                if lapStatus.lap > raceSet.num_lap
                    saveHistory(agent)
                    if !raceSet.GP_LOCAL_FLAG && !raceSet.GP_FULL_FLAG
                        saveGPData(agent)
                    end
                end
            end

            # CONTROLLER SWITCHING
            if lapStatus.lap<=1+raceSet.PF_LAP
                solvePf(mdlPf,agent)
            else                 
                if raceSet.PF_FLAG
                    savePF(agent)
                    println("Finish path following.")
                    break
                end
                GPR(agent)
                findSS(agent)
                println(agent.SS.selStates)
            	solveKin(mdlLMPC,agent)
                if !raceSet.GP_LOCAL_FLAG && !raceSet.GP_FULL_FLAG && lapStatus.it>1
                    gpDataCollect(agent)
                end
            end

            # ITERATION UPDATE AND PUBLISH
            visualUpdate(mpc_vis,agent)
            publish(vis_pub, mpc_vis)
            println("$(agent.mpcSol.sol_status) Current Lap: ", lapStatus.lap, ", It: ", lapStatus.it, " v: $(posInfo.v)")
            historyCollect(agent)
            publish(ecu_pub, cmd)
        else
            println("No estimation data received!")
        end
        rossleep(loop_rate)
    end
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
