function saveOldTraj(oldTraj::OldTrajectory,zCurr::Array{Float64},uCurr::Array{Float64},lapStatus::LapStatus,buffersize::Int64,dt::Float64)
                
                i               = lapStatus.currentIt           # current iteration number, just to make notation shorter
                zCurr_export    = zeros(buffersize,4)
                uCurr_export    = zeros(buffersize,2)
                zCurr_export    = cat(1,zCurr[1:i-1,:], [zCurr[i-1,1]+collect(1:buffersize-i+1)*dt*zCurr[i-1,4] ones(buffersize-i+1,1)*zCurr[i-1,2:4]])
                uCurr_export    = cat(1,uCurr[1:i-1,:], zeros(buffersize-i+1,2))
                costLap         = lapStatus.currentIt               # the cost of the current lap is the time it took to reach the finish line
                
                # Save all data in oldTrajectory:
                if lapStatus.currentLap == 1                        # if it's the first lap
                    oldTraj.oldTraj[:,:,1]  = zCurr_export          # ... just save everything
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
end

function InitializeParameters(mpcParams::MpcParams,trackCoeff::TrackCoeff,modelParams::ModelParams,
                                posInfo::PosInfo,oldTraj::OldTrajectory,mpcCoeff::MpcCoeff,lapStatus::LapStatus,buffersize::Int64)
    mpcParams.N                 = 10
    mpcParams.nz                = 4
    mpcParams.Q                 = [0.0,10.0,0.0,0.1]      # put weights on ey, epsi and v
    mpcParams.Q_term            = 1*[0.1,0.01,1.0]           # weights for terminal constraints (LMPC, for e_y, e_psi, and v)
    mpcParams.R                 = 0*[1.0,1.0]             # put weights on a and d_f
    mpcParams.QderivZ           = 1.0*[0,0.0,0.1,0]             # cost matrix for derivative cost of states
    mpcParams.QderivU           = 1.0*[1,1]                 # cost matrix for derivative cost of inputs
    mpcParams.vPathFollowing    = 0.6                     # reference speed for first lap of path following

    trackCoeff.nPolyCurvature   = 8                       # 4th order polynomial for curvature approximation
    trackCoeff.coeffCurvature   = zeros(trackCoeff.nPolyCurvature+1)         # polynomial coefficients for curvature approximation (zeros for straight line)
    trackCoeff.width            = 0.6                     # width of the track (0.5m)

    modelParams.u_lb            = ones(mpcParams.N,1) * [0.0  -pi/6]                    # lower bounds on steering
    modelParams.u_ub            = ones(mpcParams.N,1) * [1.2   pi/6]                  # upper bounds
    modelParams.z_lb            = ones(mpcParams.N+1,1)*[-Inf -Inf -Inf -0.1]                    # lower bounds on states
    modelParams.z_ub            = ones(mpcParams.N+1,1)*[ Inf  Inf  Inf  2.0]                    # upper bounds
    #modelParams.c0              = [0.5431, 1.2767, 2.1516, -2.4169]         # BARC-specific parameters (measured)
    modelParams.c0              = [1, 0.63, 1, 0]         # BARC-specific parameters (measured)
    modelParams.l_A             = 0.125
    modelParams.l_B             = 0.125
    modelParams.dt              = 0.1

    posInfo.s_start             = 0.0
    posInfo.s_target            = 36.84#31.62#25.62#29.491949#13.20#10.281192

    oldTraj.oldTraj             = zeros(buffersize,4,2)
    oldTraj.oldTraj[:,1,1]      = 1:buffersize
    oldTraj.oldTraj[:,1,2]      = 1:buffersize
    oldTraj.oldInput            = zeros(buffersize,2,2)
    oldTraj.oldCost             = 100*ones(Int64,2)                   # dummies for initialization

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

    n_poly_curv = trackCoeff.nPolyCurvature         # polynomial degree of curvature approximation
    
    m.mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.05))#,linear_solver="ma57",print_user_options="yes"))

    @variable( m.mdl, m.z_Ol[1:(N+1),1:4])      # z = s, ey, epsi, v
    @variable( m.mdl, m.u_Ol[1:N,1:2])
    @variable( m.mdl, 0 <= m.ParInt[1:1] <= 1)

    for i=1:2       # I don't know why but somehow the short method returns errors sometimes
        for j=1:N
            setlowerbound(m.u_Ol[j,i], modelParams.u_lb[j,i])
            setupperbound(m.u_Ol[j,i], modelParams.u_ub[j,i])
        end
    end
    for i=1:4
        for j=1:N+1
            setlowerbound(m.z_Ol[j,i], modelParams.z_lb[j,i])
            setupperbound(m.z_Ol[j,i], modelParams.z_ub[j,i])
        end
    end
    #@variable( m.mdl, 1 >= m.ParInt >= 0 )

    @NLparameter(m.mdl, m.z0[i=1:4] == z_Init[i])
    @NLconstraint(m.mdl, [i=1:4], m.z_Ol[1,i] == m.z0[i])

    @NLparameter(m.mdl, m.coeff[i=1:n_poly_curv+1] == trackCoeff.coeffCurvature[i]);

    #@NLexpression(m.mdl, m.c[i = 1:N],    m.coeff[1]*m.z_Ol[1,i]^4+m.coeff[2]*m.z_Ol[1,i]^3+m.coeff[3]*m.z_Ol[1,i]^2+m.coeff[4]*m.z_Ol[1,i]+m.coeff[5])
    @NLexpression(m.mdl, m.c[i = 1:N],    sum{m.coeff[j]*m.z_Ol[i,1]^(n_poly_curv-j+1),j=1:n_poly_curv} + m.coeff[n_poly_curv+1])
    @NLexpression(m.mdl, m.bta[i = 1:N],  atan( L_a / (L_a + L_b) * ( m.u_Ol[i,2] ) ) )
    @NLexpression(m.mdl, m.dsdt[i = 1:N], m.z_Ol[i,4]*cos(m.z_Ol[i,3]+m.bta[i])/(1-m.z_Ol[i,2]*m.c[i]))

    # System dynamics
    for i=1:N
        @NLconstraint(m.mdl, m.z_Ol[i+1,1]  == m.z_Ol[i,1] + dt*m.dsdt[i]  )                                             # s
        @NLconstraint(m.mdl, m.z_Ol[i+1,2]  == m.z_Ol[i,2] + dt*m.z_Ol[i,4]*sin(m.z_Ol[i,3]+m.bta[i])  )                     # ey
        @NLconstraint(m.mdl, m.z_Ol[i+1,3]  == m.z_Ol[i,3] + dt*(m.z_Ol[i,4]/L_a*sin(m.bta[i])-m.dsdt[i]*m.c[i])  )            # epsi
        @NLconstraint(m.mdl, m.z_Ol[i+1,4]  == m.z_Ol[i,4] + dt*(m.u_Ol[i,1] - 0.63*abs(m.z_Ol[i,4]) * m.z_Ol[i,4]))  # v
    end

end

function simModel(z::Array{Float64},u::Array{Float64},modelParams::ModelParams,trackCoeff::TrackCoeff)

    # kinematic bicycle model
    # u[1] = acceleration
    # u[2] = steering angle
    println("SIMULATING: u = $u, z0 = $z")

    dt  = modelParams.dt/10
    l_A = modelParams.l_A
    l_B = modelParams.l_B

    zNext = copy(z)
    for i=1:10
        s = zNext[1]
        c = 0
        for j=trackCoeff.nPolyCurvature:-1:0
            c += s^j * trackCoeff.coeffCurvature[trackCoeff.nPolyCurvature+1-j]
        end
        bta = atan(l_A/(l_A+l_B)*tan(u[2]))
        dsdt = zNext[4] *cos(zNext[3]+bta)/(1-zNext[2]*c)

        zNext[1] = zNext[1] + dt*dsdt                       # s
        zNext[2] = zNext[2] + dt*(zNext[4]*sin(zNext[3] + bta))     # y
        zNext[3] = zNext[3] + dt*(zNext[4]/l_A*sin(bta)-dsdt*c)        # psi
        zNext[4] = zNext[4] + dt*(u[1] - 0.63 * zNext[4]^2 * sign(zNext[4]))                     # v
    end

    return zNext
end

function extendOldTraj(oldTraj::OldTrajectory,posInfo::PosInfo,zCurr::Array{Float64,2})
    #println(size(zCurr[1:21,:]))
    #println(size(oldTraj.oldTraj[oldTraj.oldCost[1]:oldTraj.oldCost[1]+20,1:4,1]))
    for i=1:4
        for j=1:50
            oldTraj.oldTraj[oldTraj.oldCost[1]+j-1,i,1] = zCurr[j,i]
        end
    end
    oldTraj.oldTraj[oldTraj.oldCost[1]:oldTraj.oldCost[1]+49,1,1] += posInfo.s_target
end