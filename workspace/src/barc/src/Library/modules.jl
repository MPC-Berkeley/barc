#=
    File name: funtions.jl
    Author: Shuqi Xu
    Email: shuqixu@kth.se
    Julia Version: 0.4.7
=#
#=
Modules:
    ControllerHelper:
        1. find_idx(agent::Agent)
        2. trackFrame_to_xyFrame()
        3. curvature_prediction()
    SysIDFuncs:
        1. find_feature_dist(): find feature points data from pre-selected 8-figure data
        2. find_SS_dist(): find feature points data from previous laps
        3. coeff_iden_dist(): do least-mean-square SYS ID
    SafeSetFuncs:
        1. find_SS(): find safe set points for current position
    CarSim:
        1. 
        2.
        3.
        4.
        5.
    DataSavingFuncs:
        1. saveHistory(): save history data for post data analysis
        2. saveGPData(): save one-step prediction error data for Gaussian Process Regression(GPR)
=#
module TrackHelper
using RobotOS
export createTrack, Track
    function createTrack(name::ASCIIString)
        # RACING TRACK DATA
        if name == "race"
            # TRACK USED IN MY SIMULATION
            track_data=[80 0;
                        120 -pi/2;
                        80 0;
                        220 -pi*0.85;
                        105 pi/15;
                        300  pi*1.15;
                        240  -pi*0.865;
                        100 0;
                        120 -pi/2;
                        153 0;
                        120 -pi/2;
                        211 0]
        elseif name == "3110"
            # EXPERIEMENT TRACK DATA
            num = 100
            track_data=[Int(ceil(2*80)) 0;
                        Int(ceil(2*num)) pi/2;
                        Int(ceil(2*(80+47))) 0;
                        Int(ceil(2*num)) pi/2;
                        Int(ceil(2*50)) 0;
                        Int(ceil(2*num)) pi/2;
                        Int(ceil(2*4)) 0;
                        Int(ceil(2*num)) -pi/2;
                        Int(ceil(2*30)) 0;
                        Int(ceil(2*num)) pi/2;
                        Int(ceil(2*4)) 0;
                        Int(ceil(2*num)) pi/2;
                        Int(ceil(2*(71+48))) 0]
        elseif name == "basic"
            # Basic experiment track
            track_data = [Int(ceil(3*60)) 0;
                          Int(ceil(3*80)) pi/2;
                          Int(ceil(3*20)) 0;
                          Int(ceil(3*80)) pi/2;
                          Int(ceil(3*40)) -pi/10;
                          Int(ceil(3*60)) pi/5;
                          Int(ceil(3*40)) -pi/10;
                          Int(ceil(3*80)) pi/2;
                          Int(ceil(3*20)) 0;
                          Int(ceil(3*80)) pi/2;
                          Int(ceil(3*75)) 0]

            # track_data = [Int(ceil(2.8*40)) 0;
            #               Int(ceil(2.8*120)) -pi/2;
            #               Int(ceil(2.8*5)) 0;
            #               Int(ceil(2.8*120)) -pi/2;
            #               Int(ceil(2.8*80)) 0;
            #               Int(ceil(2.8*120)) -pi/2;
            #               Int(ceil(2.8*5)) 0;
            #               Int(ceil(2.8*120)) -pi/2;
            #               Int(ceil(2.8*40)) 0]
        elseif name == "MSC_lab"    
            # TRACK TO USE IN THE SMALL EXPERIMENT ROOM
            track_data = [Int(ceil(1.5*3*10)) 0;
                          Int(ceil(1.5*3*120)) pi;
                          Int(ceil(1.5*3*20)) 0;
                          Int(ceil(1.5*3*120)) pi;
                          Int(ceil(1.5*3*10)) 0]
        elseif name == "feature"
            # FEATURE TRACK DATA
            ds = 0.01
            v = 2.5    
            max_a=7.6;
            R=v^2/max_a
            max_c=1/R
            angle=(pi+pi/2)-0.105
            R_kin = 0.8
            num_kin = Int(round(angle/ ( ds/R_kin ) * 2))
            num = max(Int(round(angle/ ( ds/R ) * 2)),num_kin)
            num=Int(ceil(num*1.0))
            track_data=[num -angle;
                        num  angle]
        else
            error("Please input the correct track name")
        end
        return track_data
    end

    function add_curve(theta::Array{Float64,1},curvature::Array{Float64,1},num_point,angle::Float64,ds::Float64)
        d_theta = 0
        curve = 2*sum(1:num_point/2)#+num_point/2
        for i=1:num_point
            if i <= num_point/2
                d_theta = d_theta + angle / curve
            elseif i == num_point/2+1
                d_theta = d_theta
            else
                d_theta = d_theta - angle / curve
            end
            append!(theta,[theta[end]+d_theta])
            curv_curr = d_theta/ds
            append!(curvature,[curv_curr])
        end
        return theta,curvature
    end

    type Track
        xy::Array{Float64,2}
        idx     # not dummy idx vector, which will be useful when finding the nearest point in function "find_idx()"
        bound1xy::Array{Float64,2} # bound data are only for plotting visualization
        bound2xy::Array{Float64,2} # bound data are only for plotting visualization
        curvature::Array{Float64,1}
        max_curvature::Float64
        theta::Array{Float64,1}
        ds::Float64 # track discretization distance
        n_node::Int64 # number of points in the track
        w::Float64  # track width
        s::Float64  # track total length
        function Track(track_data::Array{Float64,2})
            # object Initialization
            track=new()

            xy=[0.0 0.0] # 1.x 2.y
            if get_param("feature_flag")
                theta=[pi/4]
            else
                theta=[0.0]
            end
                
            curvature=[0.0]
            ds      = get_param("ds")
            width   = get_param("ey")
            bound1xy = xy + width/2*[cos(theta[1]+pi/2) sin(theta[1]+pi/2)]
            bound2xy = xy - width/2*[cos(theta[1]+pi/2) sin(theta[1]+pi/2)]

            # create the track segment angle
            for i=1:size(track_data,1)
                (theta,curvature)=add_curve(theta,curvature,track_data[i,1],track_data[i,2],ds)
            end
            max_curvature=maximum(abs(curvature))
            # create the track segment by segment using the angle just calculated
            for i=2:size(theta,1) # we will have one more point at the end of the array, which is the same as the original starting point
                xy_next=[xy[end,1]+ds*cos(theta[i]) xy[end,2]+ds*sin(theta[i])]
                bound1xy_next = xy_next + width/2*[cos(theta[i]+pi/2) sin(theta[i]+pi/2)]
                bound2xy_next = xy_next - width/2*[cos(theta[i]+pi/2) sin(theta[i]+pi/2)]
                xy=vcat(xy,xy_next)
                bound1xy=vcat(bound1xy,bound1xy_next)
                bound2xy=vcat(bound2xy,bound2xy_next)
            end

            # object construction
            track.xy=xy; track.bound1xy=bound1xy; track.bound2xy=bound2xy;
            track.curvature=curvature; track.max_curvature=max_curvature;
            track.theta=theta; track.ds=ds; track.w=width
            track.n_node=size(xy,1); track.idx=1:size(xy,1)
            track.s=(track.n_node-1)*track.ds # Important: exclude the last point from the track length!
            print("Julia: $(track.s) m, $(track.n_node)")
            return track
        end
    end
end # end of module Track

module Types
using TrackHelper, JLD
using barc.msg
using RobotOS
export LapStatus,History,PosInfo,RaceSet,Agent
export SafeSet,SysID,MpcSol,MpcParams,ModelParams,GPData
    type LapStatus
        #=
        Updated lap status information
        =#
        lap::Int64
        it::Int64
        switchLap::Bool
        nextLap::Bool
        s_lapTrigger::Float64
        function LapStatus()
            lapStatus = new()
            lapStatus.it = 1
            if get_param("PF_flag")
                lapStatus.lap = 1
            else
                lapStatus.lap = 1+max(get_param("controller/Nl"),1+get_param("controller/feature_Nl"))
            end
            lapStatus.switchLap     = false
            lapStatus.nextLap       = false
            lapStatus.s_lapTrigger  = 0.3
            return lapStatus
        end
    end

    type MpcParams
        N::Int64
        dt::Float64
        vPf::Float64
        delay_a::Int64
        delay_df::Int64
        n_state::Int64
        Q::Array{Float64,1}
        R::Array{Float64,1}
        QderivZ::Array{Float64,1}
        QderivU::Array{Float64,1}
        Q_term_cost::Float64
        Q_lane::Float64
        Q_slack::Array{Float64,1}
        function MpcParams()
            mpcParams = new()
            mpcParams.N  = get_param("controller/N")
            mpcParams.dt = get_param("controller/dt")
            mpcParams.vPf = 1.0
            mpcParams.delay_a   = get_param("controller/delay_a")
            mpcParams.delay_df  = get_param("controller/delay_df")
            mpcParams.n_state   = get_param("controller/n_state")
            if get_param("sim_flag")
                mpcParams.Q             = [0.0,50.0,5.0,20.0]
                mpcParams.R             = 0*[10.0,10.0]
                mpcParams.QderivU       = 1.0*[0.01,0.5]
                mpcParams.Q_term_cost   = 0.5
                mpcParams.Q_lane        = 10.0
                if mpcParams.n_state == 6
                    mpcParams.QderivZ = 1.0*[0,0.1,0.1,2,0.1,0.0]
                    mpcParams.Q_slack = 50.0*[1.0,5.0,5.0,1.0,1.0,1.0]
                elseif mpcParams.n_state == 4
                    mpcParams.QderivZ = 1.0*[0,0.1,0.1,2]
                    mpcParams.Q_slack = 50.0*[1,1,1,1]
                end
            else
                mpcParams.Q             = [0.0,20.0,2.0,10.0]
                mpcParams.R             = 0*[10.0,10.0]
                mpcParams.QderivU       = 1*[1.0,5.0]
                mpcParams.Q_term_cost   = 0.1
                mpcParams.Q_lane        = 16
                if mpcParams.n_state == 6
                    mpcParams.QderivZ = 1.0*[0,0.1,0.1,2,0.1,0.0]
                    mpcParams.Q_slack = 50.0*[1.0,5.0,5.0,1.0,1.0,1.0]
                elseif mpcParams.n_state == 4
                    mpcParams.QderivZ = 1.0*[0,0.1,0.1,2]
                    mpcParams.Q_slack = 30.0*[1,1,1,1]
                end
            end
            return mpcParams
        end
    end

    type ModelParams
        L_a::Float64
        L_b::Float64
        m::Float64
        I_z::Float64
        c_f::Float64
        function ModelParams()
            modelParams = new()
            modelParams.L_a = get_param("L_a")
            modelParams.L_b = get_param("L_b")
            modelParams.m   = get_param("m")
            modelParams.I_z = get_param("I_z")
            modelParams.c_f = get_param("c_f")
        end # Only used for functions from CarSim module
    end

    type History
        #=
        Save history data
            MPC solution history:
                1. u: input solution history
                2. z: state solution history
            SYS ID history
                1. c_Vx: SYS ID history for Vx
                2. c_Vy: SYS ID history for Vy
                3. c_Psi: SYS ID history for psiDot
            Safe set history
                1. SS: selected safe set history
            GP history
                1. GP_vy: GP history on state vy
                2. GP_psiDot: GP history on state psiDot
            Other useful history:
                1. ax: estimated ax history in controller
                2. ay: estimated ay history in controller
        =#
        u::Array{Float64,4}
        z::Array{Float64,4}
        cost::Array{Float64,1}
        c_Vx::Array{Float64}
        c_Vy::Array{Float64}
        c_Psi::Array{Float64}
        SS::Array{Float64}
        feature_z::Array{Float64}
        feature_u::Array{Float64}
        GP_vy::Array{Float64}
        GP_psiDot::Array{Float64}
        ax::Array{Float64,2}
        ay::Array{Float64,2}
        function History(BUFFERSIZE::Int64,lapNum::Int64)
            history=new()
            N = get_param("controller/N")
            n_state = get_param("controller/n_state")
            history.c_Vx    = zeros(BUFFERSIZE,lapNum,3)
            history.c_Vy    = zeros(BUFFERSIZE,lapNum,4)
            history.c_Psi   = zeros(BUFFERSIZE,lapNum,3)
            history.SS          = zeros(BUFFERSIZE,lapNum,get_param("controller/Nl")*get_param("controller/Np"),n_state)
            history.feature_z   = zeros(BUFFERSIZE,lapNum,get_param("controller/feature_Np"),6)
            history.feature_u   = zeros(BUFFERSIZE,lapNum,get_param("controller/feature_Np"),2)
            history.GP_vy       = zeros(BUFFERSIZE,lapNum,N)
            history.GP_psiDot   = zeros(BUFFERSIZE,lapNum,N)
            if get_param("PF_flag")
                history.u       = zeros(BUFFERSIZE,lapNum,N,2)
                history.z       = zeros(BUFFERSIZE,lapNum,N+1,6)
                history.cost    = 2*ones(lapNum)
                history.ax      = zeros(BUFFERSIZE,lapNum)
                history.ay      = zeros(BUFFERSIZE,lapNum)
            else
                if get_param("sim_flag")
                    data  = load("$(homedir())/simulations/PF.jld")
                else
                    data  = load("$(homedir())/experiments/PF.jld")
                end
                history_PF = data["history"]
                history.z = history_PF.z
                history.u = history_PF.u
                history.cost = history_PF.cost
                history.ax = history_PF.ax
                history.ay = history_PF.ay
            end
            return history
        end
    end

    type SafeSet
        #=
        Save data for contructing the safe set
            Safe set data:
                1. oldSS: history states of previous laps
                2. oldCost: lap cost vector of previous laps
                3. Np: number of points selected from each lap into safe set
                4. Nl: number of previous laps to select safe set points
            Safe set result:
                1. selStates: points in the safe set
                2. stateCost: cost assigned for points in safe set
        =#
        oldSS::Array{Float64}
        oldCost::Array{Int64}
        Np::Int64
        Nl::Int64
        selStates::Array{Float64,2}
        stateCost::Array{Float64,1}
        function SafeSet(BUFFERSIZE::Int64,lapNum::Int64)
            SS = new()
            n_state     = get_param("controller/n_state")
            SS.Np   = get_param("controller/Np")
            SS.Nl   = get_param("controller/Nl")
            SS.selStates = zeros(SS.Nl*SS.Np,n_state)
            SS.stateCost = zeros(SS.Nl*SS.Np)
            if get_param("PF_flag")
                SS.oldSS    = NaN*ones(BUFFERSIZE,n_state,lapNum)
                SS.oldCost  = ones(lapNum)
            else
                if get_param("sim_flag")
                    data  = load("$(homedir())/simulations/PF.jld")
                else
                    data  = load("$(homedir())/experiments/PF.jld")
                end
                SS_PF = data["SS"]
                SS.oldSS = SS_PF.oldSS
                SS.oldCost = SS_PF.oldCost
            end
            return SS
        end
    end

    type SysID
        #=
        Save data for SYS ID and results of SYS ID
            SYS ID data:
                1. feature_Np: number of points selected from each lap
                2. feature_Nl: number of previous laps to select feature data points
                3. feature_z: feature points state data for SYS ID
                4. feature_u: feature points input data for SYS ID
            SYS ID result:
                1. c_Vx: SYS ID coefficients for state Vx
                2. c_Vy: SYS ID coefficients for state Vy
                3. c_Psi: SYS ID coefficients for state psiDot
        =#
        # Feature data for SYS ID
        feature_Np::Int64
        feature_Nl::Int64
        feature_z::Array{Float64}
        feature_u::Array{Float64}
        # SYS ID result
        c_Vx::Array{Float64}
        c_Vy::Array{Float64}
        c_Psi::Array{Float64}
        function SysID()
            sysID = new()        
            N       = get_param("controller/N")
            # Feature data for SYS ID
            sysID.feature_Np = get_param("controller/feature_Np")
            sysID.feature_Nl = get_param("controller/feature_Nl")
            sysID.feature_z = zeros(sysID.feature_Np,6)
            sysID.feature_u = zeros(sysID.feature_Np,2)
            # SYS ID result 
            sysID.c_Vx  = zeros(N,3)
            sysID.c_Vy  = zeros(N,4)
            sysID.c_Psi = zeros(N,3)
            return sysID
        end
    end

    type MpcSol
        #=
        Save result related to MPC solution
            MPC solution:
                1. u: MPC inputs
                2. z: MPC states
                3. u_prev: previvous time step MPC inputs
                4. z_prev: previvous time step MPC states
            Inputs for publishing:
                1. a_x: acceleration to publish
                2. d_f: steering to publish
                3. a_his: system acceleration delay vector to be constrained in controller
                4. df_his: system steering delay vector to be constrained in controller
            Gaussian Process compensation:
                1. GP_e_vy: GP compensation on state vy
                2. GP_e_psiDot GP compensation on state psiDot
        =#
        u::Array{Float64,2}
        z::Array{Float64,2}
        sol_status::Symbol
        u_prev::Array{Float64,2}
        z_prev::Array{Float64,2}
        a_x::Float64
        d_f::Float64
        a_his::Array{Float64,1}
        df_his::Array{Float64,1}
        function MpcSol()
            mpcSol  = new()
            N       = get_param("controller/N")
            n_state = get_param("controller/n_state")
            mpcSol.u        = zeros(N,2)
            mpcSol.z        = zeros(N+1,n_state)
            mpcSol.u_prev   = zeros(N,2)
            mpcSol.z_prev   = zeros(N+1,n_state)
            mpcSol.a_x      = 0.0
            mpcSol.d_f      = 0.0
            mpcSol.a_his    = zeros(get_param("controller/delay_a"))
            mpcSol.df_his   = zeros(get_param("controller/delay_df"))
            return mpcSol
        end
    end

    type PosInfo
        #=
        Save estiamted state infomation from pos_info msg
        =#
        s::Float64
        ey::Float64
        epsi::Float64
        v::Float64
        x::Float64
        y::Float64
        vx::Float64
        vy::Float64
        psi::Float64
        psiDot::Float64
        ax::Float64
        ay::Float64
        a::Float64
        df::Float64
        function PosInfo()
            posInfo = new()
            posInfo.s       = 0.0
            posInfo.ey      = 0.0
            posInfo.epsi    = 0.0
            posInfo.v       = 0.0
            posInfo.x       = 0.0
            posInfo.y       = 0.0
            posInfo.vx      = 0.0
            posInfo.vy      = 0.0
            posInfo.psi     = 0.0
            posInfo.psiDot  = 0.0
            posInfo.ax      = 0.0
            posInfo.ay      = 0.0
            posInfo.a       = 0.0
            posInfo.df      = 0.0
            return posInfo
        end
    end

    type RaceSet
        LMPC_LAP::Int64
        PF_LAP::Int64
        num_lap::Int64
        sim_flag::Bool
        PF_FLAG::Bool
        GP_LOCAL_FLAG::Bool
        GP_FULL_FLAG::Bool
        folder_name::ASCIIString
        file_name::ASCIIString
        function RaceSet(file_name)
            raceSet = new()
            raceSet.LMPC_LAP        = get_param("LMPC_LAP")
            raceSet.PF_LAP          = max(get_param("controller/Nl"),1+get_param("controller/feature_Nl")) # 1 is for SS selection
            raceSet.PF_FLAG         = get_param("PF_flag")
            raceSet.num_lap         = 1+raceSet.LMPC_LAP+raceSet.PF_LAP # 1 is for warm up
            raceSet.sim_flag        = get_param("sim_flag")
            raceSet.GP_LOCAL_FLAG   = get_param("controller/GP_LOCAL_FLAG")
            raceSet.GP_FULL_FLAG    = get_param("controller/GP_FULL_FLAG")
            if get_param("sim_flag")
                raceSet.folder_name = "simulations"
            else
                raceSet.folder_name = "experiments"
            end
            raceSet.file_name = file_name
            return raceSet
        end
    end

    type GPData
        #=
        Save GP data
            GP data set:
                1.feature_GP: GP feature data containing state: vx,vy,psiDot and input: a,df
                2.feature_GP_vy_e: GP feature output data for vy
                3.feature_GP_psiDot_e: GP feature output data for psiDot
                3.GP_e_vy_prepare: precomputed full GP vector for vy
                3.GP_e_psiDot_prepare: precomputed full GP vector for psiDot
            GP result:
                1. GP_vy_e: GP result data for state vy
                2. GP_psiDot_e: GP result data for state psiDot
        =#
        # GP data set
        feature_GP_z::Array{Float64}
        feature_GP_u::Array{Float64}
        feature_GP::Array{Float64}
        feature_GP_vy_e::Array{Float64}
        feature_GP_psiDot_e::Array{Float64}
        GP_e_vy_prepare::Array{Float64}
        GP_e_psiDot_prepare::Array{Float64}
        # GP result
        GP_vy_e::Array{Float64,1}
        GP_psiDot_e::Array{Float64,1}
        counter::Int64
        function GPData(file_GP_name)
            gpData = new()
            if get_param("controller/GP_LOCAL_FLAG") || get_param("controller/GP_FULL_FLAG")
                if get_param("PF_FLAG")              
                    if get_param("sim_flag")
                        data = load("$(homedir())/simulations/GP-$(file_GP_name).jld")
                    else
                        data = load("$(homedir())/experiments/GP-$(file_GP_name).jld")
                    end
                end
                num_spare         = 30 # THE NUMBER OF POINTS SELECTED FOR SPARE GP
                feature_GP_z      = data["feature_GP_z"]
                feature_GP_u      = data["feature_GP_u"]
                gpData.feature_GP_z   = feature_GP_z[1:num_spare:end,:]
                gpData.feature_GP_u   = feature_GP_u[1:num_spare:end,:]
                gpData.GP_feature     = hcat(feature_GP_z,feature_GP_u)

                feature_GP_vy_e         = data["feature_GP_vy_e"]
                feature_GP_psiDot_e     = data["feature_GP_psidot_e"]
                gpData.feature_GP_vy_e      = feature_GP_vy_e[1:num_spare:end]
                gpData.feature_GP_psiDot_e  = feature_GP_psiDot_e[1:num_spare:end]

                gpData.GP_e_vy_prepare      = GP_prepare(gpData.feature_GP_vy_e,gpData.GP_feature)
                gpData.GP_e_psiDot_prepare  = GP_prepare(gpData.feature_GP_psiDot_e,gpData.GP_feature)
            else
                n_state = get_param("controller/n_state")
                gpData.feature_GP_z         = zeros(10000,n_state)
                gpData.feature_GP_u         = zeros(10000,2)
                gpData.feature_GP_vy_e      = zeros(10000)
                gpData.feature_GP_psiDot_e  = zeros(10000)
            end
            N = get_param("controller/N")
            gpData.GP_vy_e     = zeros(N)
            gpData.GP_psiDot_e = zeros(N)
            gpData.counter   = 1
            return gpData
        end
    end

    type Agent
        #=
        Composite type containing other types related to the controller
        =#
        track::Track
        posInfo::PosInfo
        sysID::SysID
        SS::SafeSet
        lapStatus::LapStatus
        mpcSol::MpcSol
        history::History
        mpcParams::MpcParams
        gpData::GPData
        raceSet::RaceSet
        cmd::ECU
        function Agent(track::Track,posInfo::PosInfo,sysID::SysID,SS::SafeSet,lapStatus::LapStatus,mpcSol::MpcSol,
                       history::History,mpcParams::MpcParams,gpData::GPData,raceSet::RaceSet,cmd::ECU)
            agent = new()
            agent.track     = track
            agent.posInfo   = posInfo
            agent.sysID     = sysID
            agent.SS        = SS
            agent.lapStatus = lapStatus
            agent.mpcSol    = mpcSol
            agent.history   = history
            agent.mpcParams = mpcParams
            agent.gpData    = gpData
            agent.raceSet   = raceSet
            agent.cmd       = cmd
            return agent
        end
    end
end # end of module Types

module ControllerHelper
using TrackHelper
using Types, barc.msg
export curvature_prediction, lapSwitch, SE_callback, visualUpdate

    function find_idx(s::Float64,track::Track)
        idx = Int(ceil(s/track.ds)+1)
        if idx > track.n_node
            idx -= track.n_node
        end
        return idx
    end

    function trackFrame_to_xyFrame(z_sol::Array{Float64,2},track::Track)
        # Position sanity check
        n = size(z_sol,1)
        z_x = zeros(n)
        z_y = zeros(n)
        for i in 1:n
            z = z_sol[i,:]
            if z[1]>track.s
                z[1]-=track.s
            elseif z[1]<0
                z[1]+=track.s
            end
            ds=track.ds; s=z[1]; ey=z[2]; epsi=z[3]
            idx = find_idx(z[1],track)
    
            x_track=track.xy[idx,1]
            y_track=track.xy[idx,2]
            theta=track.theta[idx]
            x=x_track+ey*cos(theta+pi/2)
            y=y_track+ey*sin(theta+pi/2)

            z_x[i]=x
            z_y[i]=y
        end
        return z_x, z_y
    end

    function curvature_prediction(s::Array{Float64},track::Track)    
        curvature=zeros(length(s))
        for i=1:length(s)
            curvature[i]=track.curvature[find_idx(s[i],track)]
        end
        return curvature
    end

    function lapSwitch(agent::Agent)
        if agent.raceSet.PF_FLAG || (!agent.raceSet.PF_FLAG && agent.lapStatus.lap > 1+agent.raceSet.PF_LAP) 
            agent.SS.oldCost[agent.lapStatus.lap]   = agent.lapStatus.it-1
            agent.history.cost[agent.lapStatus.lap] = agent.lapStatus.it-1
        end
        agent.lapStatus.nextLap = false
        agent.lapStatus.lap += 1
        agent.lapStatus.it = 1
    end

    function SE_callback(msg::pos_info,agent::Agent)
        #=
        Inputs: 
            1. pos_info: ROS topic msg type
            2. lapStatus: user defined type from class.jl
            3. posInfo: user defined type from class.jl
        =#
        agent.posInfo.s       = msg.s
        agent.posInfo.ey      = msg.ey
        agent.posInfo.epsi    = msg.epsi
        agent.posInfo.v       = msg.v
        agent.posInfo.x       = msg.x
        agent.posInfo.y       = msg.y
        agent.posInfo.vx      = msg.v_x
        agent.posInfo.vy      = msg.v_y
        agent.posInfo.psi     = msg.psi
        agent.posInfo.psiDot  = msg.psiDot
        agent.posInfo.ax      = msg.a_x
        agent.posInfo.ay      = msg.a_y
        agent.posInfo.a       = msg.u_a
        agent.posInfo.df      = msg.u_df
        
        if agent.posInfo.s <= agent.lapStatus.s_lapTrigger && agent.lapStatus.switchLap
            agent.lapStatus.nextLap = true
            agent.lapStatus.switchLap = false
        elseif agent.posInfo.s > agent.lapStatus.s_lapTrigger
            agent.lapStatus.switchLap = true
        end
    end

    function visualUpdate(mpc_vis::mpc_visual,agent::Agent)
        (z_x,z_y)   = trackFrame_to_xyFrame(agent.mpcSol.z,agent.track)
        mpc_vis.z_x = z_x
        mpc_vis.z_y = z_y
        (SS_x,SS_y) = trackFrame_to_xyFrame(agent.SS.selStates,agent.track)
        mpc_vis.SS_x  = SS_x
        mpc_vis.SS_y  = SS_y
        mpc_vis.z_vx  = agent.mpcSol.z[:,4]
        mpc_vis.SS_vx = agent.SS.selStates[:,4]
        mpc_vis.z_s   = agent.mpcSol.z[:,1]
        mpc_vis.SS_s  = agent.SS.selStates[:,1]
    end
end # end of module ControllerHelper

module SysIDFuncs
using Types
export find_feature_dist, find_SS_dist
export coeff_iden_dist
    function find_feature_dist(agent::Agent)
        Np = agent.sysID.feature_Np
        iden_z      = zeros(Np,3,2)
        iden_z_plot = zeros(Np,6)
        iden_u      = zeros(Np,2)

        curr_state=hcat(z_curr[4:6]',u_curr)

        norm_state=[1 0.1 1 1 0.2] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
        # norm_state=[1 1 1 1/2 1/2] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
        dummy_state=z_feature[:,:,1]
        dummy_input=u_feature
        # cal_state=Float64[] # stored for normalization calculation
        cal_state=hcat(dummy_state,dummy_input)
        dummy_norm=zeros(size(dummy_state,1),2)

        norm_dist=(curr_state.-cal_state[:,4:8])./norm_state
        dummy_norm[:,1]=norm_dist[:,1].^2+norm_dist[:,2].^2+norm_dist[:,3].^2+norm_dist[:,4].^2+norm_dist[:,5].^2
        dummy_norm[:,2]=1:size(dummy_state,1)
        dummy_norm=sortrows(dummy_norm) # pick up the first minimum Np points
        # println(dummy_norm[1:Np,:])
        for i=1:Np
            iden_z[i,:,1]=z_feature[Int(dummy_norm[i,2]),4:6,1]
            iden_z[i,:,2]=z_feature[Int(dummy_norm[i,2]),4:6,2]
            iden_z_plot[i,:]=dummy_state[Int(dummy_norm[i,2]),:,1]
            iden_u[i,:]=dummy_input[Int(dummy_norm[i,2]),:]
        end
        return iden_z, iden_u, iden_z_plot
    end

    function find_SS_dist(agent::Agent)

        Nl=selectedStates.feature_Nl
        Np=selectedStates.feature_Np
        # Clear the safe set data of previous iteration
        iden_z=zeros(Np,3,2)
        iden_z_plot=zeros(Np,6)
        iden_u=zeros(Np,2)

        # curr_state=hcat(z_curr[1],z_curr[4:6]',u_curr')
        # norm_state=[0.5 1 0.1 1 1 0.3] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
        curr_state=hcat(z_curr[4:6]',u_curr)
        norm_state=[1 0.1 1 1 0.2] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
        dummy_state=Array{Float64}(0,6)
        dummy_input=Array{Float64}(0,2)
        cal_state=Array{Float64}(0,8) # stored for normalization calculation

        # collect out all the state and input history data
        # at maximum, Nl laps data will be collected, it was actually reshaped into the whole history
        for i=max(1,lapStatus.currentLap-Nl):max(1,(lapStatus.currentLap-1))
            dummy_state=vcat(dummy_state,reshape(solHistory.z[1:Int(solHistory.cost[i]),i,1,:],Int(solHistory.cost[i]),6))
            dummy_input=vcat(dummy_input,reshape(solHistory.u[1:Int(solHistory.cost[i]),i,1,:],Int(solHistory.cost[i]),2))
        end
        cal_state=vcat(cal_state,hcat(dummy_state,dummy_input))
        dummy_norm=zeros(size(dummy_state,1)-1,2)
        for i=1:size(dummy_state,1)-1 # The last state point will be removed
            # dummy_norm[i,1]=norm((curr_state-cal_state[i,vcat(1,4:8)]')./norm_state) # this is the state after normalization
            norm_dist=(curr_state.-cal_state[i,4:8])./norm_state
            dummy_norm[i,1]=norm_dist[1]^2+norm_dist[2]^2+norm_dist[3]^2+norm_dist[4]^2+norm_dist[5]^2
            # dummy_norm[i,1]=norm((curr_state-cal_state[i,4:8])./norm_state) # this is the state after normalization
            dummy_norm[i,2]=i
        end

        dummy_norm=sortrows(dummy_norm) # pick up the first minimum Np points

        for i=1:min(Np,size(dummy_norm,1))
            iden_z[i,:,1]=dummy_state[Int(dummy_norm[i,2]),4:6]
            iden_z[i,:,2]=dummy_state[Int(dummy_norm[i,2])+1,4:6]
            iden_z_plot[i,:]=dummy_state[Int(dummy_norm[i,2]),:]
            iden_u[i,:]=dummy_input[Int(dummy_norm[i,2]),:]
        end
        # iden_z: Npx3x2 states selected for system identification
        # iden_u: Npx2 inputs selected for system identification
        return iden_z, iden_u, iden_z_plot
    end

    function coeff_iden_dist(agent::Agent)
        z = idenStates
        u = idenInputs
        size(z,1)==size(u,1) ? nothing : error("state and input in coeff_iden() need to have the same dimensions")
        A_vx=zeros(size(z,1),6)
        A_vy=zeros(size(z,1),4)
        A_psi=zeros(size(z,1),3)

        y_vx = z[:,1,2] - z[:,1,1]
        y_vy = z[:,2,2] - z[:,2,1]
        y_psi = z[:,3,2] - z[:,3,1]

        for i=1:size(z,1)
            A_vx[i,1]=z[i,2,1]*z[i,3,1]
            A_vx[i,2]=z[i,1,1]
            A_vx[i,3]=u[i,1]
            A_vx[i,4]=z[i,3,1]/z[i,1,1]
            A_vx[i,5]=z[i,2,1]/z[i,1,1]
            A_vx[i,6]=u[i,2]
            A_vy[i,1]=z[i,2,1]/z[i,1,1]
            A_vy[i,2]=z[i,1,1]*z[i,3,1]
            A_vy[i,3]=z[i,3,1]/z[i,1,1]
            A_vy[i,4]=u[i,2]
            A_psi[i,1]=z[i,3,1]/z[i,1,1]
            A_psi[i,2]=z[i,2,1]/z[i,1,1]
            A_psi[i,3]=u[i,2]
        end
        # BACKSLASH OPERATOR WITHOUT REGULIZATION
        # c_Vx = A_vx\y_vx
        # c_Vy = A_vy\y_vy
        # c_Psi = A_psi\y_psi

        # BY IVS()
        # c_Vx = inv(A_vx'*A_vx)*A_vx'*y_vx
        # c_Vy = inv(A_vy'*A_vy)*A_vy'*y_vy
        # c_Psi = inv(A_psi'*A_psi)*A_psi'*y_psi

        # BACKSLASH WITH REGULARIZATION
        mu_Vx = zeros(6,6); mu_Vx[1,1] = 1e-5
        mu_Vy = zeros(4,4); mu_Vy[1,1] = 1e-5
        mu_Psi = zeros(3,3); mu_Psi[2,2] = 1e-5
        c_Vx = (A_vx'*A_vx+mu_Vx)\(A_vx'*y_vx)
        c_Vy = (A_vy'*A_vy+mu_Vy)\(A_vy'*y_vy)
        c_Psi = (A_psi'*A_psi+mu_Psi)\(A_psi'*y_psi)


        # println("c_Vx is $c_Vx")
        # println("c_Vx is $c_Vy")
        # println("c_Vx is $c_Psi")
        # c_Vx[1] = max(-0.3,min(0.3,c_Vx[1]))
        # c_Vy[2] = max(-1,min(1,c_Vy[2]))
        # c_Vy[3] = max(-1,min(1,c_Vy[3]))

        return c_Vx, c_Vy, c_Psi
    end
end # end of SysIDFunc module

module SafeSetFuncs
using Types, ControllerHelper
export findSS
    function findSS(agent::Agent)
        s       = agent.posInfo.s
        N       = agent.mpcParams.N
        dt      = agent.mpcParams.dt
        Nl      = agent.SS.Nl
        Np      = agent.SS.Np
        Np_here = agent.SS.Np/2
        target_s= s+agent.posInfo.v*dt*agent.mpcParams.N
        t_s     = copy(target_s)

        cost_correction = findmin(agent.SS.oldCost[agent.lapStatus.lap-Nl-1:agent.lapStatus.lap-1])[1]
        for i=1:Nl
            SS_befo=agent.SS.oldSS[:,:,agent.lapStatus.lap-i-1] # inculde those unfilled zeros spots
            SS_curr=agent.SS.oldSS[:,:,agent.lapStatus.lap-i]   # inculde those unfilled zeros spots
            SS_next=agent.SS.oldSS[:,:,agent.lapStatus.lap-i+1] # inculde those unfilled zeros spots
            all_s=SS_curr[1:Int(agent.SS.oldCost[agent.lapStatus.lap-i]),1]
            if target_s>agent.track.s
                target_s-=agent.track.s
            end
            (value_s,idx_s)=findmin(abs(all_s-target_s))
            idx_s_start = Int(idx_s-Np_here)
            idx_s_end   = Int(idx_s+Np_here-1)
            cost        = Int(agent.SS.oldCost[agent.lapStatus.lap-i])
            cost_befo   = Int(agent.SS.oldCost[agent.lapStatus.lap-i-1])
            
            if idx_s_end>cost
                SS_1 = SS_curr[idx_s_start:cost,:]
                SS_2 = SS_next[1:idx_s_end-cost,:]
                SS_2[:,1] += agent.track.s
                agent.SS.selStates[(i-1)*Np+1:i*Np,:] = vcat(SS_1,SS_2)
                agent.SS.stateCost[(i-1)*Np+1:i*Np]  = (agent.SS.Np:-1:1)+cost
            elseif idx_s_start<1
                if t_s > agent.track.s
                    SS_1 = SS_curr[cost+idx_s_start:cost,:]
                    SS_2 = SS_next[1:idx_s_end,:]
                    SS_2[:,1] += agent.track.s
                    agent.SS.selStates[(i-1)*Np+1:i*Np,:] = vcat(SS_1,SS_2)
                    agent.SS.stateCost[(i-1)*Np+1:i*Np]  = (agent.SS.Np:-1:1)+cost
                else
                    SS_1 = SS_befo[cost_befo+idx_s_start:cost_befo,:]
                    SS_2 = SS_curr[1:idx_s_end,:]
                    SS_1[:,1] -= agent.track.s
                    agent.SS.selStates[(i-1)*Np+1:i*Np,:] = vcat(SS_1,SS_2)
                    agent.SS.stateCost[(i-1)*Np+1:i*Np]  = (agent.SS.Np:-1:1)+cost
                end 
            else
                if t_s > agent.track.s
                    SS_curr[:,1] += agent.track.s
                    agent.SS.selStates[(i-1)*Np+1:i*Np,:] = SS_curr[idx_s_start:idx_s_end,:]
                    agent.SS.stateCost[(i-1)*Np+1:i*Np]  = (agent.SS.Np:-1:1)+cost
                else
                    agent.SS.selStates[(i-1)*Np+1:i*Np,:] = SS_curr[idx_s_start:idx_s_end,:]
                    agent.SS.stateCost[(i-1)*Np+1:i*Np]  = (agent.SS.Np:-1:1)+cost
                end
            end
        end
        agent.SS.stateCost[:] -= cost_correction
    end
end # end of SafeSetFunc module

module CarSim
using TrackHelper, Types
using ControllerHelper
    function car_sim_kin(agent::Agent)
        dt  = agent.mpcParams.dt
        L_a = agent.modelParams.L_a
        L_b = agent.modelParams.L_b
        z = agent.mpcSol.z[end,:]
        u = agent.mpcSol.u[end,:]

        idx = find_idx(z[1],agent.track)
        c=track.curvature[idx]

        bta = atan(L_a/(L_a+L_b)*tan(u[2]))
        dsdt = z[4]*cos(z[3]+bta)/(1-z[2]*c)

        zNext = copy(z)
        zNext[1] = z[1] + dt*dsdt                               # s
        zNext[2] = z[2] + dt*z[4] * sin(z[3] + bta)             # eY
        zNext[3] = z[3] + dt*(z[4]/L_b*sin(bta)-dsdt*c)         # ePsi
        zNext[4] = z[4] + dt*(u[1] - modelParams.c_f*z[4])      # v

        return zNext
    end

    function car_pre_dyn(z_curr::Array{Float64},u_sol::Array{Float64},track::Track,modelParams::ModelParams,outputDims::Int64)
        if size(u_sol,1)==1
            (z_dummy, Fy_dummy, a_slip_dummy)=car_sim_dyn_exact(z_curr,u_sol,track,modelParams)
            if outputDims==4
                z_dummy=s6_to_s4(z_dummy)
            end
        elseif size(u_sol,2)==2
            z_dummy=zeros(size(u_sol,1)+1,6)
            Fy_dummy=zeros(size(u_sol,1)+1,2)
            a_slip_dummy=zeros(size(u_sol,1)+1,2)

            z_dummy[1,:]=z_curr
            for i=2:size(u_sol,1)+1
                (z_dummy[i,:], Fy_dummy[i,:], a_slip_dummy[i,:])=car_sim_dyn_exact(z_dummy[i-1,:],u_sol[i-1,:],track,modelParams)
            end
            if outputDims==4
                z_dummy=s6_to_s4(z_dummy)
            end
        else
            error("Please check the u_sol dimension of function \"car_pre_dyn\" ")
        end

        return z_dummy, Fy_dummy, a_slip_dummy
    end

    function car_pre_dyn_true(z_curr::Array{Float64},u_sol::Array{Float64},track::Track,modelParams::ModelParams,outputDims::Int64)
        if size(u_sol,1)==1
            (z_dummy, Fy_dummy, a_slip_dummy)=car_sim_dyn_exact(z_curr,u_sol,track,modelParams)
            if outputDims==4
                z_dummy=s6_to_s4(z_dummy)
            end
        elseif size(u_sol,2)==2
            z_dummy=zeros(size(u_sol,1)+1,6)
            Fy_dummy=zeros(size(u_sol,1)+1,2)
            a_slip_dummy=zeros(size(u_sol,1)+1,2)

            z_dummy[1,:]=z_curr
            for i=2:size(u_sol,1)+1
                (z_dummy[i,:], Fy_dummy[i,:], a_slip_dummy[i,:])=car_sim_dyn_exact_true(z_dummy[i-1,:],u_sol[i-1,:],track,modelParams)
            end
            if outputDims==4
                z_dummy=s6_to_s4(z_dummy)
            end
        else
            error("Please check the u_sol dimension of function \"car_pre_dyn\" ")
        end

        return z_dummy, Fy_dummy, a_slip_dummy
    end

    function car_sim_dyn_exact_true(z::Array{Float64,2},u::Array{Float64,2},track::Track,modelParams::ModelParams)
        # This function uses smaller steps to achieve higher fidelity than we would achieve using longer timesteps
        z_final = copy(z)
        Fy=zeros(2); a_slip=zeros(2);
        u[1] = min(u[1],2)
        u[1] = max(u[1],-1)
        u[2] = min(u[2],0.1*pi)
        u[2] = max(u[2],-0.1*pi)
        dt=modelParams.dt; dtn=dt/10
        for i=1:10
            (z_final, Fy, a_slip)= car_sim_dyn(z_final,u,dtn,track,modelParams)
        end
        return z_final, Fy, a_slip
    end

    function car_sim_dyn_exact(z::Array{Float64,2},u::Array{Float64,2},track::Track,modelParams::ModelParams)
        # This function uses smaller steps to achieve higher fidelity than we would achieve using longer timesteps
        z_final = copy(z)
        Fy=zeros(2); a_slip=zeros(2);
        u[1] = min(u[1],2)
        u[1] = max(u[1],-1)
        u[2] = min(u[2],0.1*pi)
        u[2] = max(u[2],-0.1*pi)
        dt=modelParams.dt; dtn=dt/10
        for i=1:10
            (z_final, Fy, a_slip)= car_sim_dyn(z_final,u,dtn,track,modelParams)
        end
        return z_final, Fy, a_slip
    end

    function car_sim_dyn_true(z::Array{Float64},u::Array{Float64},dt::Float64,track::Track,modelParams::ModelParams)
        # s, ey, epsi, vx, vy, Psidot
        L_f = modelParams.l_A
        L_r = modelParams.l_B
        m   = modelParams.m
        I_z = modelParams.I_z
        c_f = modelParams.c_f   # motor drag coefficient

        a_F = 0 # front tire slip angle
        a_R = 0 # rear tire slip angle
        if abs(z[4]) >= 0.1
            a_F     = atan((z[5] + L_f*z[6])/abs(z[4])) - u[2]
            a_R     = atan((z[5] - L_r*z[6])/abs(z[4]))
        else
            warn("too low speed, not able to simulate the dynamic model")
        end
        if max(abs(a_F),abs(a_R))>30/180*pi
            # warn("Large tire angles: a_F = $a_F, a_R = $a_R, xDot = $(z[4]), d_F = $(u[2])")
        end

        FyF = -pacejka_true(a_F)
        FyR = -pacejka_true(a_R)

        idx=Int(ceil(z[1]/track.ds))+1 # correct the starting original point idx problem
        # println(z[1])
        # println(idx)
        idx>track.n_node ? idx=idx%track.n_node : nothing
        idx<=0 ? idx += track.n_node : nothing
        # println(idx)
        
        # println(track.n_node)
        c=track.curvature[idx]
        

        dsdt = (z[4]*cos(z[3]) - z[5]*sin(z[3]))/(1-z[2]*c)

        zNext = copy(z)
        zNext[1] = z[1] + dt * dsdt                                             # s
        zNext[2] = z[2] + dt * (z[4]*sin(z[3]) + z[5]*cos(z[3]))                # eY
        zNext[3] = z[3] + dt * (z[6]-dsdt*c)                                    # ePsi
        zNext[4] = z[4] + dt * (u[1] + z[5]*z[6] - c_f*z[4])                    # vx
        zNext[5] = z[5] + dt * ((FyF*cos(u[2])+FyR)/m - z[4]*z[6])              # vy
        zNext[6] = z[6] + dt * ((L_f*FyF*cos(u[2]) - L_r*FyR)/I_z)              # psiDot

        return zNext, [FyF,FyR], [a_F,a_R]
    end

    function car_sim_dyn(z::Array{Float64},u::Array{Float64},dt::Float64,track::Track,modelParams::ModelParams)
        # s, ey, epsi, vx, vy, Psidot
        L_f = modelParams.l_A
        L_r = modelParams.l_B
        m   = modelParams.m
        I_z = modelParams.I_z
        c_f = modelParams.c_f   # motor drag coefficient

        a_F = 0 # front tire slip angle
        a_R = 0 # rear tire slip angle
        if abs(z[4]) >= 0.1
            a_F     = atan((z[5] + L_f*z[6])/abs(z[4])) - u[2]
            a_R     = atan((z[5] - L_r*z[6])/abs(z[4]))
        else
            warn("too low speed, not able to simulate the dynamic model")
        end
        if max(abs(a_F),abs(a_R))>30/180*pi
            # warn("Large tire angles: a_F = $a_F, a_R = $a_R, xDot = $(z[4]), d_F = $(u[2])")
        end

        FyF = -pacejka(a_F)
        FyR = -pacejka(a_R)

        idx=Int(ceil(z[1]/track.ds))+1 # correct the starting original point idx problem
        # println(z[1])
        # println(idx)
        idx>track.n_node ? idx=idx%track.n_node : nothing
        idx<=0 ? idx += track.n_node : nothing
        # println(idx)
        
        # println(track.n_node)
        c=track.curvature[idx]
        

        dsdt = (z[4]*cos(z[3]) - z[5]*sin(z[3]))/(1-z[2]*c)

        zNext = copy(z)
        zNext[1] = z[1] + dt * dsdt                                             # s
        zNext[2] = z[2] + dt * (z[4]*sin(z[3]) + z[5]*cos(z[3]))                # eY
        zNext[3] = z[3] + dt * (z[6]-dsdt*c)                                    # ePsi
        zNext[4] = z[4] + dt * (u[1] + z[5]*z[6] - c_f*z[4])                    # vx
        zNext[5] = z[5] + dt * ((FyF*cos(u[2])+FyR)/m - z[4]*z[6])              # vy
        zNext[6] = z[6] + dt * ((L_f*FyF*cos(u[2]) - L_r*FyR)/I_z)              # psiDot

        return zNext, [FyF,FyR], [a_F,a_R]
    end

    function pacejka(a)
        # B = 1.0             # This value determines the steepness of the curve
        # C = 1.25
        B = 6.0             # This value determines the steepness of the curve
        C = 1.6
        mu = 0.8            # Friction coefficient (responsible for maximum lateral tire force)
        m = 1.98
        g = 9.81
        D = mu * m * g/2
        # C_alpha_f = D*sin.(C*atan.(B*a))
        C_alpha_f = D*sin(C*atan(B*a))
        return C_alpha_f
    end

    function car_sim_iden_tv(z::Array{Float64},u::Array{Float64},dt::Float64,agent::Agent)
        L_f = modelParams.l_A
        L_r = modelParams.l_B
        m   = modelParams.m
        I_z = modelParams.I_z
        c_f = modelParams.c_f

        c_Vx=mpcCoeff.c_Vx
        c_Vy=mpcCoeff.c_Vy
        c_Psi=mpcCoeff.c_Psi

        idx=Int(ceil(z[1]/track.ds))+1 # correct the starting original point idx problem
        idx>track.n_node ? idx=idx%track.n_node : nothing
        idx<=0 ? idx+=track.n_node : nothing

        c=track.curvature[idx]
        dsdt = (z[4]*cos(z[3]) - z[5]*sin(z[3]))/(1-z[2]*c)

        z_next=copy(z)

        z_next[1]  = z_next[1] + dt * dsdt                                # s
        z_next[2]  = z_next[2] + dt * (z[4]*sin(z[3]) + z[5]*cos(z[3]))   # eY
        z_next[3]  = z_next[3] + dt * (z[6]-dsdt*c)                       # ePsi
        z_next[4]  = z_next[4] + c_Vx[1]*z[5]*z[6] + c_Vx[2]*z[4] + c_Vx[3]*u[1]                           # vx
        z_next[5]  = z_next[5] + c_Vy[1]*z[5]/z[4] + c_Vy[2]*z[4]*z[6] + c_Vy[3]*z[6]/z[4] + c_Vy[4]*u[2]  # vy
        z_next[6]  = z_next[6] + c_Psi[1]*z[6]/z[4] + c_Psi[2]*z[5]/z[4] + c_Psi[3]*u[2]                   # psiDot
        return z_next
    end
end # end of CarSim module


module DataSavingFuncs
using JLD
using Types
export saveHistory, saveGPData, savePF
export historyCollect, gpDataCollect
    function saveHistory(agent::Agent)
        # DATA SAVING
        run_time = Dates.format(now(),"yyyy-mm-dd-H:M")
        log_path = "$(homedir())/$(agent.raceSet.folder_name)/LMPC-$(agent.raceSet.file_name)-$(run_time).jld"
        save(log_path,  "log_cvx",          agent.history.c_Vx,
                        "log_cvy",          agent.history.c_Vy,
                        "log_cpsi",         agent.history.c_Psi,
                        "GP_vy",            agent.history.GP_vy,
                        "GP_psiDot",        agent.history.GP_psiDot,
                        "history",          agent.history,
                        "track",            agent.track)
        println("Finish saving history to $log_path in controller node.")
    end

    function historyCollect(agent::Agent)
        if agent.raceSet.PF_FLAG || (!agent.raceSet.PF_FLAG && agent.lapStatus.lap > 1+agent.raceSet.PF_LAP)
            z_curr = [agent.posInfo.s, agent.posInfo.ey, agent.posInfo.epsi, agent.posInfo.vx, agent.posInfo.vy, agent.posInfo.psiDot]

            agent.history.z[agent.lapStatus.it,agent.lapStatus.lap,:,1:agent.mpcParams.n_state]=agent.mpcSol.z
            agent.history.z[agent.lapStatus.it,agent.lapStatus.lap,1,4:6]=z_curr[4:6]
            agent.history.u[agent.lapStatus.it,agent.lapStatus.lap,:,:]=agent.mpcSol.u
            if agent.mpcParams.n_state == 6
                agent.SS.oldSS[agent.lapStatus.it,:,agent.lapStatus.lap]=z_curr
            elseif agent.mpcParams.n_state == 4
                agent.SS.oldSS[agent.lapStatus.it,:,agent.lapStatus.lap]=[agent.posInfo.s, agent.posInfo.ey, agent.posInfo.epsi, agent.posInfo.v]
            end
            agent.lapStatus.it += 1
        end
        
        # SafeSet history
        if !agent.raceSet.PF_FLAG && agent.lapStatus.lap > 1+agent.raceSet.PF_LAP
            agent.history.SS[agent.lapStatus.it,agent.lapStatus.lap,:,:] = agent.SS.selStates
        end

        # SysID history
        agent.history.feature_z[agent.lapStatus.it,agent.lapStatus.lap,:,:] = agent.sysID.feature_z
        agent.history.feature_u[agent.lapStatus.it,agent.lapStatus.lap,:,:] = agent.sysID.feature_u

        # GP history
        if !agent.raceSet.GP_LOCAL_FLAG || !agent.raceSet.GP_FULL_FLAG
            agent.history.GP_vy[agent.lapStatus.it,agent.lapStatus.lap,:]       = agent.gpData.GP_vy_e
            agent.history.GP_psiDot[agent.lapStatus.it,agent.lapStatus.lap,:]   = agent.gpData.GP_psiDot_e
        end

        # previvous solution update
        agent.mpcSol.z_prev = agent.mpcSol.z
        agent.mpcSol.u_prev = agent.mpcSol.u
    end

    function saveGPData(agent::Agent)
        run_time = Dates.format(now(),"yyyy-mm-dd-H:M")
        log_path = "$(homedir())/$(agent.raceSet.folder_name)/GP-$(agent.raceSet.file_name)-$(run_time).jld"
        feature_GP_z         = data["feature_GP_z"]
        feature_GP_u         = data["feature_GP_u"]
        feature_GP_vy_e      = data["feature_GP_vy_e"]
        feature_GP_psidot_e  = data["feature_GP_psidot_e"]
        save(log_path,  "feature_GP_z",         agent.gpData.feature_GP_z,  
                        "feature_GP_u",         agent.gpData.feature_GP_u,  
                        "feature_GP_vy_e",      agent.gpData.feature_GP_vy_e, 
                        "feature_GP_psidot_e",  agent.gpData.feature_GP_psidot_e)
        println("Finish saving GP data to $log_path in controller node.")
    end

    function gpDataCollect(agent::Agent)
        agent.gpData.feature_GP_z[agent.gpData.counter,:] = agent.mpcSol.z_prev[1,:]
        agent.gpData.feature_GP_u[agent.gpData.counter,:] = agent.mpcSol.u_prev[1,:]
        if agent.mpcParams.n_state == 6
            agent.gpData.feature_GP_vy_e[agent.gpData.counter]      = agent.mpcSol.z[1,5]-mpcSol.z_prev[2,5]
            agent.gpData.feature_GP_psiDot_e[agent.gpData.counter]  = agent.mpcSol.z[1,6]-mpcSol.z_prev[2,6]
        end
        agent.gpData.counter += 1
    end

    function savePF(agent::Agent)
        log_path = "$(homedir())/$(agent.raceSet.folder_name)/PF.jld"
        save(log_path,"SS",agent.SS,"history",agent.history)
        println("Finsh saving path following data to $log_path for LMPC.")
    end
end # end of DataSaving module

module GPRFuncs
using Types
export GPR
    # FUNCTIONS FOR GP REGRESSION
    function covar_fun(z1::Array{Float64,2},z2::Array{Float64,2})
        # input: [vx,vy,psi_dot,a,df]
        z = z1-z2
        k = exp(-0.5*(0.1*z[1]^2+5*z[2]^2+5*z[3]^2+0.1*z[4]^2+5*z[5]^2))
        return k
    end

    function regre(z::Array{Float64,2},u::Array{Float64,2},e::Array{Float64,1},s::Array{Float64,2},i::Array{Float64,2})
        # find the closest local feature points
        dummy_norm = zeros(size(s,1),2)
        state = hcat(z,u)
        feature_state = hcat(s,i)
        dist = state[4:8]'.-feature_state[:,4:8]
        # We have 5 dimensions for distance calculation
        dummy_norm[:,1] = dist[:,1].^2+(5*dist[:,5]).^2#+dist[:,3].^2+dist[:,4].^2+dist[:,5].^2
        dummy_norm[:,2] = 1:size(dummy_norm,1)
        dummy_norm=sortrows(dummy_norm) # pick up the first minimum Np points

        # do local GP
        # pick the first 20 points for local GP
        num = 20
        K=zeros(num,num)
        k=zeros(num)
        y=zeros(num)
        for i = 1:num
            for j=1:num
                z = feature_state[Int(dummy_norm[i,2]),4:8] - feature_state[Int(dummy_norm[j,2]),4:8]
                K[i,j] = exp(-0.5*(0.1*z[1]^2+5*z[2]^2+5*z[3]^2+0.1*z[4]^2+5*z[5]^2))
                # K[i,j]=covar_fun(feature_state[Int(dummy_norm[i,2]),4:8],feature_state[Int(dummy_norm[j,2]),4:8])
            end
            z = feature_state[Int(dummy_norm[i,2]),4:8]-state[4:8]'
            k[i] = exp(-0.5*(0.1*z[1]^2+5*z[2]^2+5*z[3]^2+0.1*z[4]^2+5*z[5]^2))
            # k[i]=covar_fun(feature_state[Int(dummy_norm[i,2]),4:8],state[4:8]')
            y[i]=e[Int(dummy_norm[i,2])]
            # println(feature_state[Int(dummy_norm[i,2]),4:8])
        end

        local_e = k'*(K\y)
        return local_e[1]
    end

    function GP_prepare(e::Array{Float64,1},feature_state::Array{Float64,2})
        # find the closest local feature points
        num = size(feature_state,1)
        Z=zeros(num,num)
        for i = 1:num
            for j=1:num
                z = feature_state[i,4:8]-feature_state[j,4:8]
                Z[i,j] = (0.5*z[1]^2+5*z[2]^2+5*z[3]^2+0.5*z[4]^2+5*z[5]^2)
            end
            # y[i]=e[i]
        end
        K=exp(-0.5*Z)
        return K\e
    end

    function GP_full_vy(z::Array{Float64,2},u::Array{Float64,2},feature_state::Array{Float64,2},GP_prepare::Array{Float64,1})
        state = hcat(z,u)
        z = feature_state[:,4:8].-state[1,4:8]
        Z = 0.5*z[:,1].^2+5*z[:,2].^2+5*z[:,3].^2+0.5*z[:,4].^2+5*z[:,5].^2
        # Z = z[:,1].^2+z[:,2].^2+z[:,3].^2+z[:,4].^2+z[:,5].^2
        k = 0.1^2*exp(-0.5*Z)
        GP_e = k'*GP_prepare
        return GP_e[1]
    end

    function GP_full_psidot(z::Array{Float64,2},u::Array{Float64,2},feature_state::Array{Float64,2},GP_prepare::Array{Float64,1})
        state = hcat(z,u)
        z = feature_state[:,4:8].-state[1,4:8]
        Z = 0.5*z[:,1].^2+5*z[:,2].^2+5*z[:,3].^2+0.5*z[:,4].^2+5*z[:,5].^2
        # Z = z[:,1].^2+z[:,2].^2+z[:,3].^2+z[:,4].^2+z[:,5].^2
        k = 0.1^2*exp(-0.5*Z)
        GP_e = k'*GP_prepare
        return GP_e[1]
    end

    function GPR(agent::Agent)
        if agent.mpcParams.n_state == 6
            z_curr = [agent.posInfo.s agent.posInfo.ey agent.posInfo.epsi agent.posInfo.vx agent.posInfo.vy agent.posInfo.psiDot]
        elseif agent.mpcParams.n_state == 4
            z_curr = [agent.posInfo.s agent.posInfo.ey agent.posInfo.epsi agent.posInfo.v]
        end
        z_to_iden = vcat(z_curr,agent.mpcSol.z_prev[3:end,:])
        u_to_iden = vcat(agent.mpcSol.u_prev[2:end,:],agent.mpcSol.u_prev[end,:])
        for i = 1:size(u_to_iden,1)
            if agent.raceSet.GP_LOCAL_FLAG
                agent.gpData.GP_vy_e[i]      = regre(z_to_iden[i,:],u_to_iden[i,:],agent.gpData.feature_GP_vy_e,    agent.gpData.feature_GP)
                agent.gpData.GP_psiDot_e[i]  = regre(z_to_iden[i,:],u_to_iden[i,:],agent.gpData.feature_GP_psiDot_e,agent.gpData.feature_GP)
            elseif agent.raceSet.GP_FULL_FLAG
                agent.gpData.GP_vy_e[i]      = GP_full_vy(z_to_iden[i,:],    u_to_iden[i,:],agent.gpData.GP_feature,agent.gpData.GP_e_vy_prepare)
                agent.gpData.GP_psiDot_e[i]  = GP_full_psidot(z_to_iden[i,:],u_to_iden[i,:],agent.gpData.GP_feature,agent.gpData.GP_e_psi_dot_prepare)
            else
                agent.gpData.GP_vy_e[i]      = 0
                agent.gpData.GP_psiDot_e[i]  = 0
            end
        end
    end
end # end of module: GPRFuncs