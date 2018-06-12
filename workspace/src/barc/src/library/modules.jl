#=
    File name: modules.jl
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
__precompile__()
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
export SafeSet,SysID,FeatureData,MpcSol,MpcParams,ModelParams,GPData
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
                mpcParams.QderivU       = 1.0*[1.0,5.0]
                mpcParams.Q_term_cost   = 0.2
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
        dt::Float64
        m::Float64
        I_z::Float64
        c_f::Float64
        B::Float64
        C::Float64
        mu::Float64
        g::Float64
        function ModelParams()
            modelParams = new()
            modelParams.L_a = get_param("L_a")
            modelParams.L_b = get_param("L_b")
            modelParams.dt  = get_param("controller/dt")
            modelParams.m   = get_param("m")
            modelParams.I_z = get_param("I_z")
            modelParams.c_f = get_param("controller/c_f")
            modelParams.B  = get_param("simulator/B")
            modelParams.C  = get_param("simulator/C")
            modelParams.mu = get_param("simulator/mu")
            modelParams.g  = get_param("simulator/g")
            return modelParams
        end # Only used for: functions from CarSim module and DynLin controller
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
            if get_param("controller/SYS_ID_LIN_FLAG")
                history.c_Vx    = zeros(BUFFERSIZE,lapNum,5)
                history.c_Vy    = zeros(BUFFERSIZE,lapNum,4)
                history.c_Psi   = zeros(BUFFERSIZE,lapNum,4)
            else
                history.c_Vx    = zeros(BUFFERSIZE,lapNum,3)
                history.c_Vy    = zeros(BUFFERSIZE,lapNum,4)
                history.c_Psi   = zeros(BUFFERSIZE,lapNum,3)
            end
            history.SS          = zeros(BUFFERSIZE,lapNum,get_param("controller/Nl")*get_param("controller/Np"),n_state)
            history.feature_z   = zeros(BUFFERSIZE,lapNum,get_param("controller/feature_Np"),6)
            history.feature_u   = zeros(BUFFERSIZE,lapNum,get_param("controller/feature_Np"),2)
            history.GP_vy       = zeros(BUFFERSIZE,lapNum,N)
            history.GP_psiDot   = zeros(BUFFERSIZE,lapNum,N)
            history.u       = zeros(BUFFERSIZE,lapNum,N,2)
            history.z       = zeros(BUFFERSIZE,lapNum,N+1,6)
            history.cost    = 2*ones(lapNum)
            history.ax      = zeros(BUFFERSIZE,lapNum)
            history.ay      = zeros(BUFFERSIZE,lapNum)
            return history
        end
    end

    type FeatureData
        feature_z::Array{Float64,4}
        feature_u::Array{Float64,3}
        cost::Array{Float64,1}
        v_avg::Array{Float64,1}
        function FeatureData(BUFFERSIZE::Int64,num_lap::Int64)
            featureData = new()
            featureData.feature_z = zeros(BUFFERSIZE,num_lap,6,2)
            featureData.feature_u = zeros(BUFFERSIZE,num_lap,2)
            featureData.cost = ones(num_lap)
            featureData.v_avg= zeros(num_lap)
            return featureData
        end
    end

    type SafeSet
        #=
        Save data for contructing the safe set
            Safe set data:
                1. oldSS: history states of previous laps
                1. oldSS_u: history inputs of previous laps: to avoid doing PF again for different controller
                2. oldCost: lap cost vector of previous laps
                3. Np: number of points selected from each lap into safe set
                4. Nl: number of previous laps to select safe set points
            Safe set result:
                1. selStates: points in the safe set
                2. stateCost: cost assigned for points in safe set
        =#
        oldSS::Array{Float64}
        oldSS_u::Array{Float64}
        oldCost::Array{Int64}
        Np::Int64
        Nl::Int64
        selStates::Array{Float64,2}
        stateCost::Array{Float64,1}
        function SafeSet(BUFFERSIZE::Int64,lapNum::Int64)
            SS = new()
            n_state = get_param("controller/n_state")
            SS.Np   = get_param("controller/Np")
            SS.Nl   = get_param("controller/Nl")
            SS.selStates = zeros(SS.Nl*SS.Np,n_state)
            SS.stateCost = zeros(SS.Nl*SS.Np)
            if get_param("PF_flag")
                SS.oldSS    = NaN*ones(BUFFERSIZE,lapNum,6)
                SS.oldSS_u  = NaN*ones(BUFFERSIZE,lapNum,2)
                SS.oldCost  = ones(lapNum)
            else
                if get_param("sim_flag")
                    data  = load("$(homedir())/simulations/PF.jld")
                else
                    data  = load("$(homedir())/experiments/PF.jld")
                end
                SS_PF = data["SS"]
                SS.oldSS   = SS_PF.oldSS
                SS.oldSS_u = SS_PF.oldSS_u
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
        feature_z::Array{Float64,3}
        feature_u::Array{Float64,2}
        feature::Array{Float64,2}
        select_z::Array{Float64,3}
        select_u::Array{Float64,2}
        # SYS ID result
        c_Vx::Array{Float64}
        c_Vy::Array{Float64}
        c_Psi::Array{Float64}
        function SysID()
            sysID = new()        
            N = get_param("controller/N")
            # Feature data for SYS ID
            sysID.feature_Np = get_param("controller/feature_Np")
            sysID.feature_Nl = get_param("controller/feature_Nl")
            sysID.feature_z = rand(2*sysID.feature_Np,6,2)  # The size of this array will be overwritten
            sysID.feature_u = rand(2*sysID.feature_Np,2)    # The size of this array will be overwritten
            sysID.feature   = rand(2*sysID.feature_Np,8)    # The size of this array will be overwritten
            sysID.select_z  = zeros(sysID.feature_Np,6,2)
            sysID.select_u  = zeros(sysID.feature_Np,2)
            # SYS ID result 
            if get_param("controller/SYS_ID_LIN_FLAG")
                sysID.c_Vx  = zeros(N,5)    
                sysID.c_Vy  = zeros(N,4)    
                sysID.c_Psi = zeros(N,4)
            else    
                sysID.c_Vx  = zeros(N,3)
                sysID.c_Vy  = zeros(N,4)
                sysID.c_Psi = zeros(N,3)
            end
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
        feature_flag::Bool
        PF_FLAG::Bool
        TV_FLAG::Bool           # dummy attribute for controller without SYSID
        SYS_ID_LIN_FLAG::Bool   # dummy attribute for controller without SYSID
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
            raceSet.feature_flag    = get_param("feature_flag")
            raceSet.TV_FLAG         = get_param("controller/TV_FLAG")
            raceSet.SYS_ID_LIN_FLAG = get_param("controller/SYS_ID_LIN_FLAG")
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
        modelParams::ModelParams
        gpData::GPData
        raceSet::RaceSet
        cmd::ECU
        function Agent(track::Track,posInfo::PosInfo,sysID::SysID,SS::SafeSet,lapStatus::LapStatus,mpcSol::MpcSol,
                       history::History,mpcParams::MpcParams,modelParams::ModelParams,gpData::GPData,raceSet::RaceSet,cmd::ECU)
            agent = new()
            agent.track     = track
            agent.posInfo   = posInfo
            agent.sysID     = sysID
            agent.SS        = SS
            agent.lapStatus = lapStatus
            agent.mpcSol    = mpcSol
            agent.history   = history
            agent.mpcParams = mpcParams
            agent.modelParams= modelParams
            agent.gpData    = gpData
            agent.raceSet   = raceSet
            agent.cmd       = cmd
            return agent
        end

        # Agent light constructor for FeatureData collecting
        function Agent(track::Track,posInfo::PosInfo,lapStatus::LapStatus,mpcSol::MpcSol,
                       mpcParams::MpcParams,modelParams::ModelParams,raceSet::RaceSet,cmd::ECU)
            agent = new()
            agent.track     = track
            agent.posInfo   = posInfo
            agent.lapStatus = lapStatus
            agent.mpcSol    = mpcSol
            agent.mpcParams = mpcParams
            agent.modelParams= modelParams
            agent.raceSet   = raceSet
            agent.cmd       = cmd
            return agent
        end
    end
end # end of module Types

module ControllerHelper
using TrackHelper
using Types, barc.msg
import Types: FeatureData
export find_idx, curvature_prediction, lapSwitch, SE_callback, visualUpdate

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

    function lapSwitch(agent::Agent,featureData::FeatureData)
        # FEATURE DATA COST AND AVERAGE SPEED RECORDING
        cost = agent.lapStatus.it-1
        featureData.cost[agent.lapStatus.lap]   = cost
        featureData.v_avg[agent.lapStatus.lap]  = mean(featureData.feature_z[1:cost,agent.lapStatus.lap,4,1])
        # AGENT LAPSATUS UPDATE
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
        if !agent.raceSet.feature_flag
            (SS_x,SS_y) = trackFrame_to_xyFrame(agent.SS.selStates,agent.track)
            mpc_vis.SS_x  = SS_x
            mpc_vis.SS_y  = SS_y
            (z_iden_x, z_iden_y) = trackFrame_to_xyFrame(agent.sysID.select_z[:,:,1],agent.track)
            mpc_vis.z_iden_x  = z_iden_x
            mpc_vis.z_iden_y  = z_iden_y

            if !agent.raceSet.PF_FLAG
                # LapTime RELATED DATA UPDATE
                v_avg = [mean(agent.SS.oldSS[1:Int(agent.SS.oldCost[i]),i,4]) for i in 1:agent.lapStatus.lap-1]
                mpc_vis.v_avg = round(v_avg,2) 
                lapTime = [agent.track.s/v for v in v_avg]
                mpc_vis.lapTime = round(lapTime,2) 
                mpc_vis.cost = [Int(agent.SS.oldCost[i]) for i in 1:agent.lapStatus.lap-1]
                mpc_vis.lapNum = [Int(i) for i in 1:agent.lapStatus.lap-1]
            end
        end
    end
end # end of module ControllerHelper

module SysIDFuncs
using Types
export buildFeatureSet, sysIdTi, sysIdTv
    function buildFeatureSetFromHistory(agent::Agent)
        # THIS FUNCTION IS ONLY NEEDS TO BE CALLED ONCE EACH LAP
        Nl = agent.sysID.feature_Nl
        z1 = Array{Float64,2}(0,6)
        z2 = Array{Float64,2}(0,6)
        u = Array{Float64,2}(0,2)
        for i in (agent.lapStatus.lap-Nl) : (agent.lapStatus.lap-1)
            cost = Int(agent.SS.oldCost[i])
            z1 = vcat(z1,reshape(agent.SS.oldSS[1:cost-1,i,:],cost-1,6))
            z2 = vcat(z2,reshape(agent.SS.oldSS[2:cost,  i,:],cost-1,6))
            u = vcat(u,reshape(agent.SS.oldSS_u[1:cost-1,i,:],cost-1,2))
        end
        agent.sysID.feature_z = zeros(size(z1,1),6,2)
        agent.sysID.feature_z[:,:,1] = z1
        agent.sysID.feature_z[:,:,2] = z2
        agent.sysID.feature_u = u
        agent.sysID.feature   = hcat(z1,u)
    end

    function buildFeatureSetFromDataSet(agent::Agent,featureData::FeatureData)
        # by default, we select the 5 laps of closest v_avg speed to do SYS ID
        Nl = 5
        idx = hcat()
        z1 = Array{Float64,2}(0,6)
        z2 = Array{Float64,2}(0,6)
        u = Array{Float64,2}(0,2)
        for i in (agent.lapStatus.lap-Nl) : (agent.lapStatus.lap-1)
            cost = Int(agent.SS.oldCost[i])
            z1 = vcat(z1,reshape(agent.SS.oldSS[1:cost-1,i,:],cost-1,6))
            z2 = vcat(z2,reshape(agent.SS.oldSS[2:cost,  i,:],cost-1,6))
            u = vcat(u,reshape(agent.SS.oldSS_u[1:cost-1,i,:],cost-1,2))
        end
        agent.sysID.feature_z = zeros(size(z1,1),6,2)
        agent.sysID.feature_z[:,:,1] = z1
        agent.sysID.feature_z[:,:,2] = z2
        agent.sysID.feature_u = u
        agent.sysID.feature   = hcat(z1,u) 
    end

    function buildFeatureSetFromBoth(agent::Agent,featureData::FeatureData,ratio::Float64)
        # ratio::Float64 decide how much percent of feature data is selected from DataSet

    end

    function findFeature(agent::Agent,curr_state::Array{Float64})
        norm_state = [1 0.1 1 1 0.2]
        norm_idx   = zeros(size(agent.sysID.feature_z,1),2)

        norm_dist  = (curr_state.-agent.sysID.feature[1:end,4:8])./norm_state
        norm_idx[:,1] = sum(norm_dist.^2,2)
        norm_idx[:,2] = 1:size(norm_idx,1)
        norm_idx = sortrows(norm_idx)
        for i=1:agent.sysID.feature_Np
            agent.sysID.select_z[i,:,1] = agent.sysID.feature_z[Int(norm_idx[i,2]),:,1]
            agent.sysID.select_z[i,:,2] = agent.sysID.feature_z[Int(norm_idx[i,2]),:,2]
            agent.sysID.select_u[i,:]   = agent.sysID.feature_u[Int(norm_idx[i,2]),:]
        end
    end

    function coeffId(agent::Agent,ID_idx::Int64)
        A_vx = zeros(agent.sysID.feature_Np,3)
        A_vy = zeros(agent.sysID.feature_Np,4)
        A_psi= zeros(agent.sysID.feature_Np,3)

        y_vx  = agent.sysID.select_z[:,4,2] - agent.sysID.select_z[:,4,1]
        y_vy  = agent.sysID.select_z[:,5,2] - agent.sysID.select_z[:,5,1]
        y_psi = agent.sysID.select_z[:,6,2] - agent.sysID.select_z[:,6,1]

        for i=1:agent.sysID.feature_Np
            A_vx[i,1]   = agent.sysID.select_z[i,5,1]*agent.sysID.select_z[i,6,1]
            A_vx[i,2]   = agent.sysID.select_z[i,4,1]
            A_vx[i,3]   = agent.sysID.select_u[i,1]
            A_vy[i,1]   = agent.sysID.select_z[i,5,1]/agent.sysID.select_z[i,4,1]
            A_vy[i,2]   = agent.sysID.select_z[i,4,1]*agent.sysID.select_z[i,6,1]
            A_vy[i,3]   = agent.sysID.select_z[i,6,1]/agent.sysID.select_z[i,4,1]
            A_vy[i,4]   = agent.sysID.select_u[i,2]
            A_psi[i,1]  = agent.sysID.select_z[i,6,1]/agent.sysID.select_z[i,4,1]
            A_psi[i,2]  = agent.sysID.select_z[i,5,1]/agent.sysID.select_z[i,4,1]
            A_psi[i,3]  = agent.sysID.select_u[i,2]
        end
        # BACKSLASH OPERATOR WITHOUT REGULIZATION
        # c_Vx = A_vx\y_vx
        # c_Vy = A_vy\y_vy
        # c_Psi = A_psi\y_psi

        # BACKSLASH WITH REGULARIZATION
        mu_Vx = zeros(3,3); mu_Vx[1,1] = 1e-5
        mu_Vy = zeros(4,4); mu_Vy[1,1] = 1e-5
        mu_Psi = zeros(3,3); mu_Psi[2,2] = 1e-5
        agent.sysID.c_Vx[ID_idx,:]  = (A_vx'*A_vx+mu_Vx)\(A_vx'*y_vx)
        agent.sysID.c_Vy[ID_idx,:]  = (A_vy'*A_vy+mu_Vy)\(A_vy'*y_vy)
        agent.sysID.c_Psi[ID_idx,:] = (A_psi'*A_psi+mu_Psi)\(A_psi'*y_psi)
    end

    function coeffIdLin(agent::Agent,ID_idx::Int64)
        A_vx = zeros(agent.sysID.feature_Np,5)
        A_vy = zeros(agent.sysID.feature_Np,4)
        A_psi= zeros(agent.sysID.feature_Np,4)

        y_vx  = agent.sysID.select_z[:,4,2] - agent.sysID.select_z[:,4,1]
        y_vy  = agent.sysID.select_z[:,5,2] - agent.sysID.select_z[:,5,1]
        y_psi = agent.sysID.select_z[:,6,2] - agent.sysID.select_z[:,6,1]

        for i=1:agent.sysID.feature_Np
            A_vx[i,1]   = agent.sysID.select_z[i,4,1]
            A_vx[i,2]   = agent.sysID.select_z[i,5,1]
            A_vx[i,3]   = agent.sysID.select_z[i,6,1]
            A_vx[i,4]   = agent.sysID.select_u[i,1]
            A_vx[i,5]   = agent.sysID.select_u[i,2]
            A_vy[i,1]   = agent.sysID.select_z[i,4,1]
            A_vy[i,2]   = agent.sysID.select_z[i,5,1]
            A_vy[i,3]   = agent.sysID.select_z[i,6,1]
            A_vy[i,4]   = agent.sysID.select_u[i,2]
            A_psi[i,1]  = agent.sysID.select_z[i,4,1]
            A_psi[i,2]  = agent.sysID.select_z[i,5,1]
            A_psi[i,3]  = agent.sysID.select_z[i,6,1]
            A_psi[i,4]  = agent.sysID.select_u[i,2]
        end
        # BACKSLASH OPERATOR WITHOUT REGULIZATION
        agent.sysID.c_Vx[ID_idx,:]  = A_vx\y_vx
        agent.sysID.c_Vy[ID_idx,:]  = A_vy\y_vy
        agent.sysID.c_Psi[ID_idx,:] = A_psi\y_psi

        # BACKSLASH WITH REGULARIZATION
        # mu_Vx = zeros(5,5); mu_Vx[1,1] = 1e-5
        # mu_Vy = zeros(4,4); mu_Vy[1,1] = 1e-5
        # mu_Psi = zeros(4,4); mu_Psi[2,2] = 1e-5
        # agent.sysID.c_Vx[ID_idx,:]  = (A_vx'*A_vx+mu_Vx)\(A_vx'*y_vx)
        # agent.sysID.c_Vy[ID_idx,:]  = (A_vy'*A_vy+mu_Vy)\(A_vy'*y_vy)
        # agent.sysID.c_Psi[ID_idx,:] = (A_psi'*A_psi+mu_Psi)\(A_psi'*y_psi)
    end

    function sysIdTi(agent::Agent)
        curr_state = [agent.posInfo.vx agent.posInfo.vy agent.posInfo.psiDot agent.mpcSol.u[1,1] agent.mpcSol.u[1,2]]
        findFeature(agent,curr_state)
        if agent.raceSet.SYS_ID_LIN_FLAG
            coeffIdLin(agent,1)
        else
            coeffId(agent,1)
        end
        for i in 2:agent.mpcParams.N
            agent.sysID.c_Vx[i,:]   = agent.sysID.c_Vx[1,:] 
            agent.sysID.c_Vy[i,:]   = agent.sysID.c_Vy[1,:]
            agent.sysID.c_Psi[i,:]  = agent.sysID.c_Psi[1,:]
        end 
    end

    function sysIdTv(agent::Agent)
        curr_state = [agent.posInfo.vx agent.posInfo.vy agent.posInfo.psiDot agent.mpcSol.u[1,1] agent.mpcSol.u[1,2]]
        state = vcat(curr_state,hcat(agent.mpcSol.z_prev[3:end,4:6],agent.mpcSol.u_prev[2:end,:]))
        for i in 1:agent.mpcParams.N
            findFeature(agent,state[i,:])
            if agent.raceSet.SYS_ID_LIN_FLAG
                coeffIdLin(agent,1)
            else
                coeffId(agent,1)
            end
        end
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
            SS_befo=agent.SS.oldSS[:,agent.lapStatus.lap-i-1,1:agent.mpcParams.n_state] # inculde those unfilled zeros spots
            SS_curr=agent.SS.oldSS[:,agent.lapStatus.lap-i,  1:agent.mpcParams.n_state] # inculde those unfilled zeros spots
            SS_next=agent.SS.oldSS[:,agent.lapStatus.lap-i+1,1:agent.mpcParams.n_state] # inculde those unfilled zeros spots
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
import SysIDFuncs: findFeature, coeffId
    function carSimKin(z::Array{Float64},u::Array{Float64},track::Track,modelParams::ModelParams)
        # THIS FUNCTION IS FOR FEATURE DATA COLLECTING CONTROLLER
        dt  = modelParams.dt
        L_a = modelParams.L_a
        L_b = modelParams.L_b
        c_f = modelParams.c_f

        idx = find_idx(z[1],track)
        c   = track.curvature[idx]

        bta = atan(L_a/(L_a+L_b)*tan(u[2]))
        dsdt = z[4]*cos(z[3]+bta)/(1-z[2]*c)

        zNext = copy(z)
        zNext[1] = z[1] + dt*dsdt                       # s
        zNext[2] = z[2] + dt*z[4] * sin(z[3] + bta)     # eY
        zNext[3] = z[3] + dt*(z[4]/L_b*sin(bta)-dsdt*c) # ePsi
        zNext[4] = z[4] + dt*(u[1] - c_f*z[4])          # v
        return zNext
    end

    function carPreDyn(z_curr::Array{Float64},u::Array{Float64,2},track::Track,modelParams::ModelParams)
            z_out = zeros(size(u,1)+1,6)
            z_out[1,:] = z_curr
            for i=2:size(u,1)+1
                z_out[i,:] = carSimDynExact(z_out[i-1,:],u[i-1,:],track,modelParams)
            end
        return z_out
    end

    function carSimDynExact(z_curr::Array{Float64},u::Array{Float64},track::Track,modelParams::ModelParams)
        z_next = copy(z_curr)
        dtn = modelParams.dt/10
        for i=1:10
            z_next = carSimDyn(z_next,u,dtn,track,modelParams)
        end
        return z_next
    end

    function carSimDyn(z::Array{Float64},u::Array{Float64},dt::Float64,track::Track,modelParams::ModelParams)
        # s, ey, epsi, vx, vy, Psidot
        L_f = modelParams.L_a
        L_r = modelParams.L_a
        m   = modelParams.m
        I_z = modelParams.I_z
        c_f = modelParams.c_f

        a_F = 0
        a_R = 0
        if abs(z[4]) >= 0.2
            a_F = atan((z[5] + L_f*z[6])/abs(z[4])) - u[2]
            a_R = atan((z[5] - L_r*z[6])/abs(z[4]))
        end

        FyF = -pacejka(a_F)
        FyR = -pacejka(a_R)

        idx = find_idx(z[1],track)
        c   = track.curvature[idx]
        
        dsdt = (z[4]*cos(z[3]) - z[5]*sin(z[3]))/(1-z[2]*c)

        zNext = copy(z)
        zNext[1] = z[1] + dt * dsdt                                # s
        zNext[2] = z[2] + dt * (z[4]*sin(z[3]) + z[5]*cos(z[3]))   # eY
        zNext[3] = z[3] + dt * (z[6]-dsdt*c)                       # ePsi
        zNext[4] = z[4] + dt * (u[1] + z[5]*z[6] - c_f*z[4])       # vx
        zNext[5] = z[5] + dt * ((FyF*cos(u[2])+FyR)/m - z[4]*z[6]) # vy
        zNext[6] = z[6] + dt * ((L_f*FyF*cos(u[2]) - L_r*FyR)/I_z) # psiDot
        return zNext
    end

    function pacejka(a)
        B = 6.0
        C = 1.6
        mu = 0.8
        m = 1.98
        g = 9.81
        D = mu * m * g/2
        C_alpha_f = D*sin(C*atan(B*a))
        return C_alpha_f
    end

    function carPreId(z_curr::Array{Float64},u::Array{Float64},agent::Agent)
        z_out = zeros(agent.mpcParams.N+1,6)
        z_out[1,:] = z_curr
        for i in 1:agent.mpcParams.N
            findFeature(agent,hcat(z_out[i,4:6],u[i,:]))
            coeffId(agent,i)
            z_out[i+1,:] = carSimId(z_out[i,:],u[i,:],agent,i)
        end
        return z_out[:,1:4]
    end

    function carSimId(z::Array{Float64},u::Array{Float64},agent::Agent,i::Int64)
        if z[4] == 0
            z[4] = 0.1
        end # PREVENT NaN WHEN COMPILING FUNCTION
        L_f = agent.modelParams.L_a
        L_r = agent.modelParams.L_b
        m   = agent.modelParams.m
        I_z = agent.modelParams.I_z
        c_f = agent.modelParams.c_f
        dt  = agent.mpcParams.dt

        c_Vx =agent.sysID.c_Vx
        c_Vy =agent.sysID.c_Vy
        c_Psi=agent.sysID.c_Psi

        idx = find_idx(z[1],agent.track)
        c = agent.track.curvature[idx]
        dsdt = (z[4]*cos(z[3]) - z[5]*sin(z[3]))/(1-z[2]*c)

        z_next=copy(z)
        z_next[1]  = z[1] + dt * dsdt                                # s
        z_next[2]  = z[2] + dt * (z[4]*sin(z[3]) + z[5]*cos(z[3]))   # eY
        z_next[3]  = z[3] + dt * (z[6]-dsdt*c)                       # ePsi
        z_next[4]  = z[4] + c_Vx[i,1]*z[5]*z[6]  + c_Vx[i,2]*z[4]       + c_Vx[i,3]*u[1]                       # vx
        z_next[5]  = z[5] + c_Vy[i,1]*z[5]/z[4]  + c_Vy[i,2]*z[4]*z[6]  + c_Vy[i,3]*z[6]/z[4] + c_Vy[i,4]*u[2] # vy
        z_next[6]  = z[6] + c_Psi[i,1]*z[6]/z[4] + c_Psi[i,2]*z[5]/z[4] + c_Psi[i,3]*u[2]                      # psiDot
        return z_next
    end
end # end of CarSim module


module DataSavingFuncs
using JLD
using Types
import Types: FeatureData
export saveHistory, saveGPData, savePF
export historyCollect, gpDataCollect
    function saveHistory(agent::Agent)
        # DATA SAVING
        run_time = Dates.format(now(),"yyyy-mm-dd-H:M")
        log_path = "$(homedir())/$(agent.raceSet.folder_name)/LMPC-$(agent.raceSet.file_name)-$(run_time).jld"
        if agent.lapStatus.lap > agent.raceSet.num_lap
            # FINISH ALL RACING LAPS
            save(log_path,  "z",         agent.history.z,
                            "u",         agent.history.u,
                            "SS_z",      agent.history.SS,
                            "sysID_z",   agent.history.feature_z,
                            "sysID_u",   agent.history.feature_u,
                            "log_cvx",   agent.history.c_Vx,
                            "log_cvy",   agent.history.c_Vy,
                            "log_cpsi",  agent.history.c_Psi,
                            "GP_vy",     agent.history.GP_vy,
                            "GP_psiDot", agent.history.GP_psiDot,
                            "track",     agent.track,
                            "cost",      agent.history.cost)
        else
            # CALCULATE FINISHED ITERATION OF CURRENT LAP
            agent.history.cost[agent.lapStatus.lap] = agent.lapStatus.it-1
            # RACE IS KILLED
            save(log_path,  "z",         agent.history.z[:,1:agent.lapStatus.lap,:,:],
                            "u",         agent.history.u[:,1:agent.lapStatus.lap,:,:],
                            "SS_z",      agent.history.SS[:,1:agent.lapStatus.lap,:,:],
                            "sysID_z",   agent.history.feature_z[:,1:agent.lapStatus.lap,:,:],
                            "sysID_u",   agent.history.feature_u[:,1:agent.lapStatus.lap,:,:],
                            "log_cvx",   agent.history.c_Vx[:,1:agent.lapStatus.lap,:],
                            "log_cvy",   agent.history.c_Vy[:,1:agent.lapStatus.lap,:],
                            "log_cpsi",  agent.history.c_Psi[:,1:agent.lapStatus.lap,:],
                            "GP_vy",     agent.history.GP_vy[:,1:agent.lapStatus.lap],
                            "GP_psiDot", agent.history.GP_psiDot[:,1:agent.lapStatus.lap],
                            "track",     agent.track,
                            "cost",      agent.history.cost[1:agent.lapStatus.lap])
        end
        println("Finish saving history to $log_path in controller node.")
    end

    function historyCollect(agent::Agent)
        if agent.raceSet.PF_FLAG || (!agent.raceSet.PF_FLAG && agent.lapStatus.lap > 1+agent.raceSet.PF_LAP)
            z_curr = [agent.posInfo.s, agent.posInfo.ey, agent.posInfo.epsi, agent.posInfo.vx, agent.posInfo.vy, agent.posInfo.psiDot]

            agent.history.z[agent.lapStatus.it,agent.lapStatus.lap,:,1:agent.mpcParams.n_state]=agent.mpcSol.z
            agent.history.z[agent.lapStatus.it,agent.lapStatus.lap,1,4:6]= z_curr[4:6]
            agent.history.u[agent.lapStatus.it,agent.lapStatus.lap,:,:]  = agent.mpcSol.u
            agent.SS.oldSS[agent.lapStatus.it,agent.lapStatus.lap,:]     = z_curr
            agent.SS.oldSS_u[agent.lapStatus.it,agent.lapStatus.lap,:]   = agent.mpcSol.u[1,:]
        end
        
        # SafeSet history
        if !agent.raceSet.PF_FLAG && agent.lapStatus.lap > 1+agent.raceSet.PF_LAP
            agent.history.SS[agent.lapStatus.it,agent.lapStatus.lap,:,:] = agent.SS.selStates
        end

        # SysID history
        agent.history.feature_z[agent.lapStatus.it,agent.lapStatus.lap,:,:] = agent.sysID.select_z[:,:,1]
        agent.history.feature_u[agent.lapStatus.it,agent.lapStatus.lap,:,:] = agent.sysID.select_u

        # GP history
        if !agent.raceSet.GP_LOCAL_FLAG || !agent.raceSet.GP_FULL_FLAG
            agent.history.GP_vy[agent.lapStatus.it,agent.lapStatus.lap,:]     = agent.gpData.GP_vy_e
            agent.history.GP_psiDot[agent.lapStatus.it,agent.lapStatus.lap,:] = agent.gpData.GP_psiDot_e
        end

        # ITERATION COUNTER UPDATE
        agent.lapStatus.it += 1

        # previvous solution update
        agent.mpcSol.z_prev[:] = agent.mpcSol.z
        agent.mpcSol.u_prev[:] = agent.mpcSol.u
    end

    function saveGPData(agent::Agent)
        run_time = Dates.format(now(),"yyyy-mm-dd-H:M")
        log_path = "$(homedir())/$(agent.raceSet.folder_name)/GP-$(agent.raceSet.file_name)-$(run_time).jld"
        save(log_path,  "feature_GP_z",         agent.gpData.feature_GP_z[1:agent.gpData.counter,:],  
                        "feature_GP_u",         agent.gpData.feature_GP_u[1:agent.gpData.counter,:],  
                        "feature_GP_vy_e",      agent.gpData.feature_GP_vy_e[1:agent.gpData.counter], 
                        "feature_GP_psidot_e",  agent.gpData.feature_GP_psiDot_e[1:agent.gpData.counter])
        println("Finish saving GP data to $log_path in controller node.")
    end

    function gpDataCollect(agent::Agent)
        agent.gpData.feature_GP_z[agent.gpData.counter,:] = agent.mpcSol.z_prev[1,:]
        agent.gpData.feature_GP_u[agent.gpData.counter,:] = agent.mpcSol.u_prev[1,:]
        if agent.mpcParams.n_state == 6
            agent.gpData.feature_GP_vy_e[agent.gpData.counter]      = agent.mpcSol.z[1,5]-agent.mpcSol.z_prev[2,5]
            agent.gpData.feature_GP_psiDot_e[agent.gpData.counter]  = agent.mpcSol.z[1,6]-agent.mpcSol.z_prev[2,6]
        end
        agent.gpData.counter += 1
    end

    function savePF(agent::Agent)
        log_path = "$(homedir())/$(agent.raceSet.folder_name)/PF.jld"
        save(log_path,"SS",agent.SS)
        println("Finsh saving path following data to $log_path for LMPC.")
    end

    function saveFeatureData(agent::Agent,featureData::FeatureData)
        log_path = "$(homedir())/$(agent.raceSet.folder_name)/FD.jld"
        save(log_path,"featureData",featureData)
        println("Finsh saving feature data to $log_path.")
    end

    function featureDataCollect(agent::Agent,featureData::FeatureData)
        if agent.lapStatus.it > 1
            z_curr = [agent.posInfo.s, agent.posInfo.ey, agent.posInfo.epsi, agent.posInfo.vx, agent.posInfo.vy, agent.posInfo.psiDot]
            featureData.feature_z[agent.lapStatus.it,agent.lapStatus.lap,:,1]   = z_curr
            featureData.feature_z[agent.lapStatus.it-1,agent.lapStatus.lap,:,2] = z_curr
            featureData.feature_u[agent.lapStatus.it,agent.lapStatus.lap,:]     = agent.mpcSol.u[1,:]
        end

        # ITERATION COUNTER UPDATE
        agent.lapStatus.it += 1

        # previvous solution update
        agent.mpcSol.z_prev[:] = agent.mpcSol.z
        agent.mpcSol.u_prev[:] = agent.mpcSol.u
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