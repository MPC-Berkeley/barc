function saveOldTraj(oldTraj::OldTrajectory,zCurr::Array{Float64},uCurr::Array{Float64},lapStatus::LapStatus,buffersize::Int64,dt::Float64)
                
                i               = lapStatus.currentIt           # current iteration number, just to make notation shorter
                zCurr_export    = zeros(buffersize,4)
                uCurr_export    = zeros(buffersize,2)
                zCurr_export    = cat(1,zCurr[1:i-1,:], [zCurr[i-1,1]+collect(1:buffersize-i+1)*dt*zCurr[i-1,4] ones(buffersize-i+1,1)*zCurr[i-1,2:4]])
                uCurr_export    = cat(1,uCurr[1:i-1,:], zeros(buffersize-i+1,2))
                costLap         = lapStatus.currentIt               # the cost of the current lap is the time it took to reach the finish line
                # Save all data in oldTrajectory:
                if lapStatus.currentLap == 1                        # if it's the first lap
                    oldTraj.oldTraj[:,:,1]  = zCurr_export         # ... just save everything
                    oldTraj.oldInput[:,:,1] = uCurr_export
                    oldTraj.oldTraj[:,:,2]  = zCurr_export
                    oldTraj.oldInput[:,:,2] = uCurr_export
                    oldTraj.oldCost = [costLap,costLap]
                else                                                # idea: always copy the new trajectory in the first array!
                    if oldTraj.oldCost[1] < oldTraj.oldCost[2]      # if the first old traj is better than the second
                        oldTraj.oldTraj[:,:,2]  = oldTraj.oldTraj[:,:,1]    # ... copy the first in the second
                        oldTraj.oldInput[:,:,2] = oldTraj.oldInput[:,:,1]   # ... same for the input
                        oldTraj.oldCost[2] = oldTraj.oldCost[1]
                    end
                    oldTraj.oldTraj[:,:,1]  = zCurr_export                 # ... and write the new traj in the first
                    oldTraj.oldInput[:,:,1] = uCurr_export
                    oldTraj.oldCost[1] = costLap
                end
                #println(size(save_oldTraj))
                #println(size(oldTraj.oldTraj))
                #save_oldTraj[:,:,:,lapStatus.currentLap] = oldTraj.oldTraj[:,:,:]

                
end

function InitializeParameters(mpcParams::MpcParams,trackCoeff::TrackCoeff,modelParams::ModelParams,
                                posInfo::PosInfo,oldTraj::OldTrajectory,mpcCoeff::MpcCoeff,lapStatus::LapStatus,buffersize::Int64)
    mpcParams.N                 = 10
    mpcParams.nz                = 4
    mpcParams.Q                 = [0.0,10.0,10.0,1.0]         # put weights on ey, epsi and v
    mpcParams.R                 = [0.0,0.0]                 # put weights on ey, epsi and v
    mpcParams.vPathFollowing    = 0.5
    mpcParams.QderivZ           = 0.0*[1,1,1,1]             # cost matrix for derivative cost of states
    mpcParams.QderivU           = 0.1*[1,1]                 # cost matrix for derivative cost of inputs
    mpcParams.vPathFollowing    = 0.5

    trackCoeff.coeffCurvature   = [0.0,0.0,0.0,0.0,0.0]         # polynomial coefficients for curvature approximation (zeros for straight line)
    trackCoeff.nPolyCurvature   = 4                   # 4th order polynomial for curvature approximation
    trackCoeff.width            = 1.0                 # width of the track (0.5m)

    modelParams.u_lb            = [-1.0 -pi/6]' * ones(1,mpcParams.N)                    # lower bounds on steering
    modelParams.u_ub            = [1.0  pi/6]' * ones(1,mpcParams.N)                    # upper bounds
    modelParams.z_lb            = [-Inf -trackCoeff.width/2 -Inf -Inf]' * ones(1,mpcParams.N+1)                    # lower bounds on states
    modelParams.z_ub            = [Inf   trackCoeff.width/2  Inf  Inf]' * ones(1,mpcParams.N+1)                    # upper bounds
    #modelParams.c0              = [0.5431, 1.2767, 2.1516, -2.4169]         # BARC-specific parameters (measured)
    modelParams.c0              = [1, 0.63, 1, 0]         # BARC-specific parameters (measured)
    modelParams.l_A             = 0.125
    modelParams.l_B             = 0.125
    modelParams.dt              = 0.1

    posInfo.s_start             = 0
    posInfo.s_target            = 16.281192

    oldTraj.oldTraj             = zeros(buffersize,4,2)
    oldTraj.oldInput            = zeros(buffersize,2,2)

    mpcCoeff.order              = 5
    mpcCoeff.coeffCost          = zeros(mpcCoeff.order+1,2)
    mpcCoeff.coeffConst         = zeros(mpcCoeff.order+1,2,3) # nz-1 because no coeff for s
    mpcCoeff.pLength            = 4*mpcParams.N        # small values here may lead to numerical problems since the functions are only approximated in a short horizon

    lapStatus.currentLap        = 1         # initialize lap number
    lapStatus.currentIt         = 0         # current iteration in lap
end

function InitializeModel(m::MpcModel,mpcParams::MpcParams,modelParams::ModelParams,trackCoeff::TrackCoeff,z_Init::Array{Float64,1})

    dt   = modelParams.dt
    L_a  = modelParams.l_A
    L_b  = modelParams.l_B
    c0   = modelParams.c0
    u_lb = modelParams.u_lb
    u_ub = modelParams.u_ub
    z_lb = modelParams.z_lb
    z_ub = modelParams.z_ub

    N    = mpcParams.N
    
    m.mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.1))#,linear_solver="ma57",print_user_options="yes"))

    @variable( m.mdl, m.z_Ol[1:4,1:(N+1)])      # z = s, ey, epsi, v
    @variable( m.mdl, m.u_Ol[1:2,1:N])

    for i=1:2       # I don't know why but somehow the short method returns errors sometimes
        for j=1:N
            setlowerbound(m.u_Ol[i,j], modelParams.u_lb[i,j])
            setupperbound(m.u_Ol[i,j], modelParams.u_ub[i,j])
        end
    end
    #@variable( m.mdl, 1 >= m.ParInt >= 0 )

    @NLparameter(m.mdl, m.z0[i=1:4] == z_Init[i])
    @NLconstraint(m.mdl, [i=1:4], m.z_Ol[i,1] == m.z0[i])

    @NLparameter(m.mdl, m.coeff[i=1:length(trackCoeff.coeffCurvature)] == trackCoeff.coeffCurvature[i]);

    @NLexpression(m.mdl, m.c[i = 1:N],    m.coeff[1]*m.z_Ol[1,i]^4+m.coeff[2]*m.z_Ol[1,i]^3+m.coeff[3]*m.z_Ol[1,i]^2+m.coeff[4]*m.z_Ol[1,i]+m.coeff[5])
    @NLexpression(m.mdl, m.bta[i = 1:N],  atan( L_a / (L_a + L_b) * tan( m.u_Ol[2,i] ) ) )
    @NLexpression(m.mdl, m.dsdt[i = 1:N], m.z_Ol[4,i]*cos(m.z_Ol[3,i]+m.bta[i])/(1-m.z_Ol[2,i]*m.c[i]))

    # System dynamics
    for i=1:N
        @NLconstraint(m.mdl, m.z_Ol[1,i+1]  == m.z_Ol[1,i] + dt*m.dsdt[i]  )                                             # s
        @NLconstraint(m.mdl, m.z_Ol[2,i+1]  == m.z_Ol[2,i] + dt*m.z_Ol[4,i]*sin(m.z_Ol[3,i]+m.bta[i])  )                     # ey
        @NLconstraint(m.mdl, m.z_Ol[3,i+1]  == m.z_Ol[3,i] + dt*(m.z_Ol[4,i]/L_a*sin(m.bta[i])-m.dsdt[i]*m.c[i])  )            # epsi
        @NLconstraint(m.mdl, m.z_Ol[4,i+1]  == m.z_Ol[4,i] + dt*(c0[1]*m.u_Ol[1,i] - c0[2]*abs(m.z_Ol[4,i]) * m.z_Ol[4,i]))  # v
    end

end