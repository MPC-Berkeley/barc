type MpcModel
    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    coeff::Array{JuMP.NonlinearParameter,1}
    c_Vx::Array{JuMP.NonlinearParameter,1}
    c_Vy::Array{JuMP.NonlinearParameter,1}
    c_Psi::Array{JuMP.NonlinearParameter,1}
    coeffTermConst::Array{JuMP.NonlinearParameter,3}
    coeffTermCost::Array{JuMP.NonlinearParameter,2}

    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}
    ParInt::JuMP.Variable

    laneCost::JuMP.NonlinearExpression
    constZTerm::JuMP.NonlinearExpression
    costZTerm::JuMP.NonlinearExpression
    derivCost::JuMP.NonlinearExpression
    controlCost::JuMP.NonlinearExpression
    costZ::JuMP.NonlinearExpression

    uPrev::Array{JuMP.NonlinearParameter,2}

    function MpcModel(mpcParams::MpcParams,mpcCoeff::MpcCoeff,modelParams::ModelParams,trackCoeff::TrackCoeff)
        m = new()
        dt   = modelParams.dt
        L_a  = modelParams.l_A
        L_b  = modelParams.l_B
        c0   = modelParams.c0
        u_lb = modelParams.u_lb
        u_ub = modelParams.u_ub
        z_lb = modelParams.z_lb
        z_ub = modelParams.z_ub

        N               = mpcParams.N
        Q               = mpcParams.Q
        Q_term          = mpcParams.Q_term
        R               = mpcParams.R
        order           = mpcCoeff.order       # polynomial order of terminal constraints and cost approximation
        ey_max          = trackCoeff.width/2

        QderivZ         = mpcParams.QderivZ::Array{Float64,1}
        QderivU         = mpcParams.QderivU::Array{Float64,1}
        Q_term_cost     = mpcParams.Q_term_cost::Float64
        delay_df        = mpcParams.delay_df
        delay_a         = mpcParams.delay_a

        acc_f           = 1.0

        n_poly_curv = trackCoeff.nPolyCurvature         # polynomial degree of curvature approximation
        
        # Path following mode:
        # Create function-specific parameters
        v_ref       = mpcParams.vPathFollowing
        z_Ref::Array{Float64,2}
        z_Ref       = cat(2,v_ref*ones(N+1,1),zeros(N+1,5))       # Reference trajectory: path following -> stay on line and keep constant velocity
        u_Ref       = zeros(N,2)

        mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.09))#,check_derivatives_for_naninf="yes"))#,linear_solver="ma57",print_user_options="yes"))

        @variable( mdl, z_Ol[1:(N+1),1:7])
        @variable( mdl, u_Ol[1:N,1:2])
        @variable( mdl, 0 <= ParInt <= 1)
        #@variable( mdl, eps[1:2] >= 0) # eps for soft lane constraints
        @variable( mdl, eps[1:N+1] >= 0) # eps for soft lane constraints

        z_lb_6s = ones(mpcParams.N+1,1)*[0.1 -Inf -Inf -Inf -Inf -Inf -Inf]                      # lower bounds on states
        z_ub_6s = ones(mpcParams.N+1,1)*[3.5  Inf Inf  Inf  Inf  Inf Inf]                      # upper bounds
        u_lb_6s = ones(mpcParams.N,1) * [-1.0  -0.3]                                         # lower bounds on steering
        u_ub_6s = ones(mpcParams.N,1) * [2.0   0.3]                                         # upper bounds

        for i=1:2
            for j=1:N
                setlowerbound(u_Ol[j,i], u_lb_6s[j,i])
                setupperbound(u_Ol[j,i], u_ub_6s[j,i])
            end
        end
        for i=1:7
            for j=1:N+1
                setlowerbound(z_Ol[j,i], z_lb_6s[j,i])
                setupperbound(z_Ol[j,i], z_ub_6s[j,i])
            end
        end

        @NLparameter(mdl, z0[i=1:7] == 0)
        @NLparameter(mdl, coeff[i=1:n_poly_curv+1] == 0)
        @NLparameter(mdl, c_Vx[i=1:3]  == 0)
        @NLparameter(mdl, c_Vy[i=1:4]  == 0)
        @NLparameter(mdl, c_Psi[i=1:3] == 0)
        @NLparameter(mdl, coeffTermConst[i=1:order+1,j=1:2,k=1:5] == 0)
        @NLparameter(mdl, coeffTermCost[i=1:order+1,j=1:2] == 0)
        @NLparameter(mdl, uPrev[1:10,1:2] == 0)

        # Conditions for first solve:
        setvalue(z0[1],1)
        setvalue(c_Vx[3],0.1)

        @NLconstraint(mdl, [i=1:7], z_Ol[1,i] == z0[i])
        #@NLconstraint(mdl, [i=1:N+1], z_Ol[i,5] <=  ey_max + eps[1])
        #@NLconstraint(mdl, [i=1:N+1], z_Ol[i,5] >= -ey_max - eps[2])
        @NLconstraint(mdl, [i=1:N+1], z_Ol[i,5] <= ey_max + eps[i])
        @NLconstraint(mdl, [i=1:N+1], z_Ol[i,5] >= -ey_max - eps[i])

        @NLexpression(mdl, c[i = 1:N], sum{coeff[j]*z_Ol[i,6]^(n_poly_curv-j+1),j=1:n_poly_curv} + coeff[n_poly_curv+1])
        @NLexpression(mdl, dsdt[i = 1:N], (z_Ol[i,1]*cos(z_Ol[i,4]) - z_Ol[i,2]*sin(z_Ol[i,4]))/(1-z_Ol[i,5]*c[i]))
        
        println("Initializing model...")

        # System dynamics
        for i=1:N
            if i<=delay_df
                @NLconstraint(mdl, z_Ol[i+1,2]  == z_Ol[i,2] + c_Vy[1]*z_Ol[i,2]/z_Ol[i,1] + c_Vy[2]*z_Ol[i,1]*z_Ol[i,3] + c_Vy[3]*z_Ol[i,3]/z_Ol[i,1] + c_Vy[4]*uPrev[delay_df+1-i,2]) # yDot
                @NLconstraint(mdl, z_Ol[i+1,3]  == z_Ol[i,3] + c_Psi[1]*z_Ol[i,3]/z_Ol[i,1] + c_Psi[2]*z_Ol[i,2]/z_Ol[i,1] + c_Psi[3]*uPrev[delay_df+1-i,2])                            # psiDot
            else
                @NLconstraint(mdl, z_Ol[i+1,2]  == z_Ol[i,2] + c_Vy[1]*z_Ol[i,2]/z_Ol[i,1] + c_Vy[2]*z_Ol[i,1]*z_Ol[i,3] + c_Vy[3]*z_Ol[i,3]/z_Ol[i,1] + c_Vy[4]*u_Ol[i-delay_df,2]) # yDot
                @NLconstraint(mdl, z_Ol[i+1,3]  == z_Ol[i,3] + c_Psi[1]*z_Ol[i,3]/z_Ol[i,1] + c_Psi[2]*z_Ol[i,2]/z_Ol[i,1] + c_Psi[3]*u_Ol[i-delay_df,2])                            # psiDot
            end
            if i<=delay_a
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + dt*(uPrev[delay_a+1-i,1] - 0.5*z_Ol[i,1]))
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*u_Ol[i,1])                              # xDot
                @NLconstraint(mdl, z_Ol[i+1,7]  == z_Ol[i,7] + dt*(uPrev[delay_a+1-i,1]-z_Ol[i,7])*acc_f)
            else
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*u_Ol[i,1])                              # xDot
                @NLconstraint(mdl, z_Ol[i+1,7]  == z_Ol[i,7] + dt*(u_Ol[i-delay_a,1]-z_Ol[i,7])*acc_f)
            end
            #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*z_Ol[i,7])                               # xDot
            #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + dt*(z_Ol[i,7] - 0.5*z_Ol[i,1]))                               # xDot
            @NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2]*z_Ol[i,3] + c_Vx[2]*z_Ol[i,1] + c_Vx[3]*z_Ol[i,7]) 
            @NLconstraint(mdl, z_Ol[i+1,4]  == z_Ol[i,4] + dt*(z_Ol[i,3]-dsdt[i]*c[i]))                                                                                 # ePsi
            @NLconstraint(mdl, z_Ol[i+1,5]  == z_Ol[i,5] + dt*(z_Ol[i,1]*sin(z_Ol[i,4])+z_Ol[i,2]*cos(z_Ol[i,4])))                                                      # eY
            @NLconstraint(mdl, z_Ol[i+1,6]  == z_Ol[i,6] + dt*dsdt[i]  )                                                                                                # s
        end
        # @NLconstraint(mdl, u_Ol[1,1]-uPrev[1,1] <= 0.05)
        # @NLconstraint(mdl, u_Ol[1,1]-uPrev[1,1] >= -0.2)
        # for i=1:N-1 # Constraints on u:
        #     @NLconstraint(mdl, u_Ol[i+1,1]-u_Ol[i,1] <= 0.05)
        #     @NLconstraint(mdl, u_Ol[i+1,1]-u_Ol[i,1] >= -0.2)
        # end

        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] <= 0.06)
        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] >= -0.06)
        for i=1:N-1 # Constraints on u:
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] <= 0.06)
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] >= -0.06)
        end

        # Cost functions

        # Derivative cost
        # ---------------------------------
        @NLexpression(mdl, derivCost, sum{QderivZ[j]*(sum{(z_Ol[i,j]-z_Ol[i+1,j])^2,i=1:N}),j=1:6} +
                                          QderivU[1]*((uPrev[1,1]-u_Ol[1,1])^2+sum{(u_Ol[i,1]-u_Ol[i+1,1])^2,i=1:N-delay_a-1})+
                                          QderivU[2]*((uPrev[1,2]-u_Ol[1,2])^2+sum{(u_Ol[i,2]-u_Ol[i+1,2])^2,i=1:N-delay_df-1}))

        # Lane cost
        # ---------------------------------
        #@NLexpression(mdl, laneCost, sum{100000*eps[i]+1000*eps[i]^2,i=1:2})
        @NLexpression(mdl, laneCost, sum{10*eps[i]+100*eps[i]^2,i=2:N+1})
        
        # Lane cost
        # ---------------------------------
        #@NLexpression(mdl, laneCost, 100*sum{z_Ol[i,5]^2*((0.5+0.5*tanh(10*(z_Ol[i,5]-ey_max))) + (0.5-0.5*tanh(10*(z_Ol[i,5]+ey_max)))),i=1:N+1})

        # Control Input cost
        # ---------------------------------
        @NLexpression(mdl, controlCost, R[1]*sum{(u_Ol[i,1])^2,i=1:N-delay_a}+
                                        R[2]*sum{(u_Ol[i,2])^2,i=1:N-delay_df})

        # Terminal constraints (soft), starting from 2nd lap
        # ---------------------------------
        @NLexpression(mdl, constZTerm, sum{Q_term[j]*(ParInt*(sum{coeffTermConst[i,1,j]*z_Ol[N+1,6]^(order+1-i),i=1:order}+coeffTermConst[order+1,1,j])+
                                            (1-ParInt)*(sum{coeffTermConst[i,2,j]*z_Ol[N+1,6]^(order+1-i),i=1:order}+coeffTermConst[order+1,2,j])-z_Ol[N+1,j])^2,j=1:5})
        
        # Terminal cost
        # ---------------------------------
        # The value of this cost determines how fast the algorithm learns. The higher this cost, the faster the control tries to reach the finish line.
        @NLexpression(mdl, costZTerm, 1/5*(Q_term_cost*(ParInt*(sum{coeffTermCost[i,1]*z_Ol[N+1,6]^(order+1-i),i=1:order}+coeffTermCost[order+1,1])+
                                      (1-ParInt)*(sum{coeffTermCost[i,2]*z_Ol[N+1,6]^(order+1-i),i=1:order}+coeffTermCost[order+1,2]))))
        
        # State cost (only for path following mode)
        # ---------------------------------
        #@NLexpression(mdl, costZ, 0.5*sum{Q[i]*sum{(z_Ol[j,i]-z_Ref[j,i])^2,j=1:N+1},i=1:6})    # Follow trajectory
        #@NLexpression(mdl, costZ, 0.5*sum{(Q[1]*(sqrt(z_Ol[j,1]^2+z_Ol[j,2]^2)-1.0)^2 + Q[4]*z_Ol[j,4]^2 + Q[5]*z_Ol[j,5]^2),j=2:N+1})

        @NLexpression(mdl, costZ, 0*QderivU[1]*sum{z_Ol[i,7],i=1:N})
        # Solve model once
        @NLobjective(mdl, Min, derivCost + constZTerm + costZTerm + laneCost + N*Q_term_cost)
        #@NLobjective(mdl, Min, derivCost + costZ)
        sol_stat=solve(mdl)
        println("Finished solve 1: $sol_stat")
        sol_stat=solve(mdl)
        println("Finished solve 2: $sol_stat")
        m.mdl = mdl
        m.z0 = z0
        m.coeff = coeff
        m.z_Ol = z_Ol
        m.u_Ol = u_Ol
        m.c_Vx = c_Vx
        m.c_Vy = c_Vy
        m.c_Psi = c_Psi
        m.ParInt = ParInt
        m.uPrev = uPrev

        m.coeffTermCost = coeffTermCost
        m.coeffTermConst = coeffTermConst

        m.derivCost = derivCost
        m.controlCost = controlCost
        m.laneCost = laneCost
        m.constZTerm = constZTerm
        m.costZTerm  = costZTerm
        m.costZ  = costZ
        return m
    end
end

type MpcModel_pF
    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    coeff::Array{JuMP.NonlinearParameter,1}
    z_Ref::Array{JuMP.NonlinearParameter,2}

    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}

    derivCost::JuMP.NonlinearExpression
    costZ::JuMP.NonlinearExpression
    controlCost::JuMP.NonlinearExpression

    uPrev::Array{JuMP.NonlinearParameter,2}

    function MpcModel_pF(mpcParams_pF::MpcParams,modelParams::ModelParams,trackCoeff::TrackCoeff)
        println("Starting creation of pf model")
        m = new()
        dt          = modelParams.dt
        L_a         = modelParams.l_A
        L_b         = modelParams.l_B
        c0          = modelParams.c0
        u_lb        = modelParams.u_lb
        u_ub        = modelParams.u_ub
        z_lb        = modelParams.z_lb
        z_ub        = modelParams.z_ub

        N           = mpcParams_pF.N
        Q           = mpcParams_pF.Q
        R           = mpcParams_pF.R
        QderivZ     = mpcParams_pF.QderivZ::Array{Float64,1}
        QderivU     = mpcParams_pF.QderivU::Array{Float64,1}
        delay_df    = mpcParams_pF.delay_df::Int64
        delay_a     = mpcParams_pF.delay_a::Int64

        println("prediction h= ",N)

        v_ref       = mpcParams_pF.vPathFollowing

        acc_f       = 1.0

        n_poly_curv = trackCoeff.nPolyCurvature         # polynomial degree of curvature approximation

        # Create function-specific parameters
        z_ref::Array{Float64,2}
        z_ref       = cat(2,zeros(N+1,3),v_ref*ones(N+1,1))       # Reference trajectory: path following -> stay on line and keep constant velocity
        u_Ref       = zeros(N,2)

        # Create Model
        mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.09))#,linear_solver="ma57",print_user_options="yes"))

        # Create variables (these are going to be optimized)
        @variable( mdl, z_Ol[1:(N+1),1:5], start = 0)          # z = s, ey, epsi, v
        @variable( mdl, u_Ol[1:N,1:2], start = 0)

        # Set bounds
        z_lb_4s = ones(mpcParams_pF.N+1,1)*[-Inf -Inf -Inf -0.5]                  # lower bounds on states
        z_ub_4s = ones(mpcParams_pF.N+1,1)*[ Inf  Inf  Inf  1.5]                  # upper bounds
        u_lb_4s = ones(mpcParams_pF.N,1) * [0  -0.3]                            # lower bounds on steering
        u_ub_4s = ones(mpcParams_pF.N,1) * [1.2   0.3]                            # upper bounds

        for i=1:2
            for j=1:N
                setlowerbound(u_Ol[j,i], u_lb_4s[j,i])
                setupperbound(u_Ol[j,i], u_ub_4s[j,i])
            end
        end
        # for i=1:4
        #     for j=1:N+1
        #         setlowerbound(z_Ol[j,i], z_lb_4s[j,i])
        #         setupperbound(z_Ol[j,i], z_ub_4s[j,i])
        #     end
        # end

        @NLparameter(mdl, z_Ref[1:N+1,1:4] == 0)
        @NLparameter(mdl, z0[i=1:5] == 0)
        @NLparameter(mdl, uPrev[1:10,1:2] == 0)

        @NLparameter(mdl, coeff[i=1:n_poly_curv+1] == 0)

        @NLexpression(mdl, c[i = 1:N], sum{coeff[j]*z_Ol[i,1]^(n_poly_curv-j+1),j=1:n_poly_curv} + coeff[n_poly_curv+1])

        # System dynamics
        setvalue(z0[4],v_ref)
        @NLconstraint(mdl, [i=1:5], z_Ol[1,i] == z0[i])         # initial condition
        for i=1:N
            if i<=delay_df
                @NLexpression(mdl, bta[i],  atan( L_a / (L_a + L_b) * tan( uPrev[delay_df+1-i,2] ) ) )
            else
                @NLexpression(mdl, bta[i],  atan( L_a / (L_a + L_b) * tan( u_Ol[i-delay_df,2] ) ) )
            end
            if i<=delay_a
                #@NLconstraint(mdl, z_Ol[i+1,4] == z_Ol[i,4] + dt*(uPrev[delay_a+1-i,1] - 0.5*z_Ol[i,4]))  # v
                @NLconstraint(mdl, z_Ol[i+1,5] == z_Ol[i,5] + dt*(uPrev[delay_a+1-i,1] - z_Ol[i,5])*acc_f)  # v
            else
                #@NLconstraint(mdl, z_Ol[i+1,4] == z_Ol[i,4] + dt*(u_Ol[i-delay_a,1] - 0.5*z_Ol[i,4]))     # v
                @NLconstraint(mdl, z_Ol[i+1,5] == z_Ol[i,5] + dt*(u_Ol[i-delay_a,1] - z_Ol[i,5])*acc_f)     # v
            end

            @NLexpression(mdl, dsdt[i], z_Ol[i,4]*cos(z_Ol[i,3]+bta[i])/(1-z_Ol[i,2]*c[i]))
            @NLconstraint(mdl, z_Ol[i+1,1] == z_Ol[i,1] + dt*dsdt[i]  )                                                # s
            @NLconstraint(mdl, z_Ol[i+1,2] == z_Ol[i,2] + dt*z_Ol[i,4]*sin(z_Ol[i,3]+bta[i])  )                        # ey
            @NLconstraint(mdl, z_Ol[i+1,3] == z_Ol[i,3] + dt*(z_Ol[i,4]/L_a*sin(bta[i])-dsdt[i]*c[i])  )               # epsi
            @NLconstraint(mdl, z_Ol[i+1,4] == z_Ol[i,4] + dt*(z_Ol[i,5] - 0.5*z_Ol[i,4]))  # v
        end

        # Cost definitions
        # Derivative cost
        # ---------------------------------
        @NLexpression(mdl, derivCost, sum{QderivZ[j]*(sum{(z_Ol[i,j]-z_Ol[i+1,j])^2,i=1:N}),j=1:4} +
                                            QderivU[1]*((uPrev[1,1]-u_Ol[1,1])^2+sum{(u_Ol[i,1]-u_Ol[i+1,1])^2,i=1:N-delay_a-1})+
                                            QderivU[2]*((uPrev[1,2]-u_Ol[1,2])^2+sum{(u_Ol[i,2]-u_Ol[i+1,2])^2,i=1:N-delay_df-1}))

        # Control Input cost
        # ---------------------------------
        @NLexpression(mdl, controlCost, 0.5*R[1]*sum{(u_Ol[i,1])^2,i=1:N-delay_a}+
                                        0.5*R[2]*sum{(u_Ol[i,2])^2,i=1:N-delay_df})

        # State cost
        # ---------------------------------
        @NLexpression(mdl, costZ, 0.5*sum{Q[i]*sum{(z_Ol[j,i]-z_Ref[j,i])^2,j=2:N+1},i=1:4})    # Follow trajectory

        # Objective function
        @NLobjective(mdl, Min, costZ + derivCost + controlCost)

        # create first artificial solution (for warm start)
        for i=1:N+1
            setvalue(z_Ol[i,:],[(i-1)*dt*v_ref 0 0 v_ref 0])
        end
        for i=1:N
            setvalue(u_Ol[i,:],[0.15 0])
        end
        # First solve
        sol_stat=solve(mdl)
        println("Finished solve 1: $sol_stat")
        sol_stat=solve(mdl)
        println("Finished solve 2: $sol_stat")
        
        m.mdl = mdl
        m.z0 = z0
        m.z_Ref = z_Ref
        m.coeff = coeff
        m.z_Ol = z_Ol
        m.u_Ol = u_Ol
        m.uPrev = uPrev
        m.derivCost = derivCost
        m.costZ = costZ
        m.controlCost = controlCost
        return m
    end
end

type MpcModel_convhull

    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    coeff::Array{JuMP.NonlinearParameter,1}
    selStates::Array{JuMP.NonlinearParameter,2}
    statesCost::Array{JuMP.NonlinearParameter,1}
    c_Vx::Array{JuMP.NonlinearParameter,1}
    c_Vy::Array{JuMP.NonlinearParameter,1}
    c_Psi::Array{JuMP.NonlinearParameter,1}
    uPrev::Array{JuMP.NonlinearParameter,2}


    eps_lane::Array{JuMP.Variable,1}
    #eps_alpha::Array{JuMP.Variable,1}
    #eps_vel::Array{JuMP.Variable,1}
    alpha::Array{JuMP.Variable,1}
    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}

    
    dsdt::Array{JuMP.NonlinearExpression,1}
    c::Array{JuMP.NonlinearExpression,1}

    derivCost::JuMP.NonlinearExpression
    controlCost::JuMP.NonlinearExpression
    laneCost::JuMP.NonlinearExpression
    terminalCost::JuMP.NonlinearExpression
    slackVx::JuMP.NonlinearExpression
    slackVy::JuMP.NonlinearExpression
    slackPsidot::JuMP.NonlinearExpression
    slackEpsi::JuMP.NonlinearExpression
    slackEy::JuMP.NonlinearExpression
    slackS::JuMP.NonlinearExpression
    #slackCost::JuMP.NonlinearExpression
    #velocityCost::JuMP.NonlinearExpression

    function MpcModel_convhull(mpcParams::MpcParams,mpcCoeff::MpcCoeff,modelParams::ModelParams,trackCoeff::TrackCoeff,selectedStates::SelectedStates)
                             

        m = new()

        #### Initialize parameters

        dt         = modelParams.dt                # time step
        L_a        = modelParams.l_A               # distance from CoM of the car to the front wheels
        L_b        = modelParams.l_B               # distance from CoM of the car to the rear wheels
        u_lb       = modelParams.u_lb              # lower bounds for the control inputs
        u_ub       = modelParams.u_ub              # upper bounds for the control inputs
        z_lb       = modelParams.z_lb              # lower bounds for the states
        z_ub       = modelParams.z_ub              # upper bounds for the states
        c0         = modelParams.c0


        ey_max      = trackCoeff.width/2           # bound for the state ey (distance from the center track). It is set as half of the width of the track for obvious reasons
        n_poly_curv = trackCoeff.nPolyCurvature    # polynomial degree for curvature approximation
        v_max       = 3                            # maximum allowed velocity

        v_ref      = mpcParams.vPathFollowing              # reference velocity for the path following 
        N          = mpcParams.N                           # Prediction horizon
        QderivZ    = mpcParams.QderivZ::Array{Float64,1}   # weights for the derivative cost on the states
        QderivU    = mpcParams.QderivU::Array{Float64,1}   # weights for the derivative cost on the control inputs
        R          = mpcParams.R::Array{Float64,1}         # weights on the control inputs
        Q          = mpcParams.Q::Array{Float64,1}         # weights on the states for path following
        Q_lane     = mpcParams.Q_lane::Float64             # weight on the soft constraint on the lane
        #Q_vel      = mpcParams.Q_vel::Float64              # weight on the soft constraint for the max velocity
        delay_df   = mpcParams.delay_df
        delay_a    = mpcParams.delay_a
        Q_slack    = mpcParams.Q_slack

        println("prediction horizon= ",N)


        Np         = selectedStates.Np::Int64              # how many states to select
        Nl         = selectedStates.Nl::Int64              # how many previous laps to select

        acc_f           = 1.0


        mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.09))#,check_derivatives_for_naninf="yes"))#,linear_solver="ma57",print_user_options="yes"))

        @variable( mdl, z_Ol[1:(N+1),1:7])
        @variable( mdl, u_Ol[1:N,1:2])
        @variable( mdl, eps_lane[1:N+1] >= 0)   # eps for soft lane constraints
        @variable( mdl, alpha[1:Nl*Np] >= 0)    # coefficients of the convex hull
       # @variable( mdl, eps_alpha[1:6] >=0)     # eps for soft constraint on alpha
       #@variable( mdl, eps_vel[1:N+1]>=0)      # eps for soft constraint on velocity



        z_lb_6s = ones(mpcParams.N+1,1)*[0.1 -Inf -Inf -Inf -Inf -Inf -Inf]                      # lower bounds on states
        z_ub_6s = ones(mpcParams.N+1,1)*[3.5  Inf Inf  Inf  Inf  Inf Inf]                      # upper bounds
        u_lb_6s = ones(mpcParams.N,1) * [-1.3  -0.3]                                         # lower bounds on steering
        u_ub_6s = ones(mpcParams.N,1) * [2.0   0.3]                                         # upper bounds

        for i=1:2
            for j=1:N
                setlowerbound(u_Ol[j,i], u_lb_6s[j,i])
                setupperbound(u_Ol[j,i], u_ub_6s[j,i])
            end
        end
        for i=1:7
            for j=1:N+1
                setlowerbound(z_Ol[j,i], z_lb_6s[j,i])
                setupperbound(z_Ol[j,i], z_ub_6s[j,i])
            end
        end

        @NLparameter(mdl, z0[i=1:7] == 0)
        @NLparameter(mdl, coeff[i=1:n_poly_curv+1] == 0)
        @NLparameter(mdl, c_Vx[i=1:3]  == 0)
        @NLparameter(mdl, c_Vy[i=1:4]  == 0)
        @NLparameter(mdl, c_Psi[i=1:3] == 0)
        @NLparameter(mdl, uPrev[1:10,1:2] == 0)
        @NLparameter(mdl, selStates[1:Nl*Np,1:6] == 0)                                 # states from the previous trajectories selected in "convhullStates"
        @NLparameter(mdl, statesCost[1:Nl*Np] == 0)                                    # costs of the states selected in "convhullStates"

        
        # Conditions for first solve:
        setvalue(z0[1],1)
        setvalue(c_Vx[3],0.1)

        @NLconstraint(mdl, [i=1:7], z_Ol[1,i] == z0[i])

        @NLconstraint(mdl, [i=2:N+1], z_Ol[i,5] <= ey_max + eps_lane[i])
        @NLconstraint(mdl, [i=2:N+1], z_Ol[i,5] >= -ey_max - eps_lane[i])
        #@NLconstraint(mdl,[i = 1:(N+1)], z_Ol[i,4] <= v_max + eps_vel[i] )      # soft constraint on maximum velocity
        @NLconstraint(mdl, sum{alpha[i],i=1:Nl*Np} == 1)                        # constraint on the coefficients of the convex hull

        #for n = 1:6
            #@NLconstraint(mdl,z_Ol[N+1,n] == sum{alpha[j]*selStates[j,n],j=1:Nl*Np})  # terminal constraint
            #@NLconstraint(mdl,z_Ol[N+1,n] >= sum{alpha[j]*selStates[j,n],j=1:Nl*Np}-eps_alpha[n])  
            #@NLconstraint(mdl,z_Ol[N+1,n] <= sum{alpha[j]*selStates[j,n],j=1:Nl*Np}+eps_alpha[n])
        #end  

        @NLexpression(mdl, c[i = 1:N], sum{coeff[j]*z_Ol[i,6]^(n_poly_curv-j+1),j=1:n_poly_curv} + coeff[n_poly_curv+1])
        @NLexpression(mdl, dsdt[i = 1:N], (z_Ol[i,1]*cos(z_Ol[i,4]) - z_Ol[i,2]*sin(z_Ol[i,4]))/(1-z_Ol[i,5]*c[i]))
        
        println("Initializing model...")

        # System dynamics
        for i=1:N
            if i<=delay_df
                @NLconstraint(mdl, z_Ol[i+1,2]  == z_Ol[i,2] + c_Vy[1]*z_Ol[i,2]/z_Ol[i,1] + c_Vy[2]*z_Ol[i,1]*z_Ol[i,3] + c_Vy[3]*z_Ol[i,3]/z_Ol[i,1] + c_Vy[4]*uPrev[delay_df+1-i,2]) # yDot
                @NLconstraint(mdl, z_Ol[i+1,3]  == z_Ol[i,3] + c_Psi[1]*z_Ol[i,3]/z_Ol[i,1] + c_Psi[2]*z_Ol[i,2]/z_Ol[i,1] + c_Psi[3]*uPrev[delay_df+1-i,2])                            # psiDot
            else
                @NLconstraint(mdl, z_Ol[i+1,2]  == z_Ol[i,2] + c_Vy[1]*z_Ol[i,2]/z_Ol[i,1] + c_Vy[2]*z_Ol[i,1]*z_Ol[i,3] + c_Vy[3]*z_Ol[i,3]/z_Ol[i,1] + c_Vy[4]*u_Ol[i-delay_df,2]) # yDot
                @NLconstraint(mdl, z_Ol[i+1,3]  == z_Ol[i,3] + c_Psi[1]*z_Ol[i,3]/z_Ol[i,1] + c_Psi[2]*z_Ol[i,2]/z_Ol[i,1] + c_Psi[3]*u_Ol[i-delay_df,2])                            # psiDot
            end
            if i<=delay_a
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + dt*(uPrev[delay_a+1-i,1] - 0.5*z_Ol[i,1]))
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*u_Ol[i,1])                              # xDot
                @NLconstraint(mdl, z_Ol[i+1,7]  == z_Ol[i,7] + dt*(uPrev[delay_a+1-i,1]-z_Ol[i,7])*acc_f)
            else
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*u_Ol[i,1])                              # xDot
                @NLconstraint(mdl, z_Ol[i+1,7]  == z_Ol[i,7] + dt*(u_Ol[i-delay_a,1]-z_Ol[i,7])*acc_f)
            end
            #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*z_Ol[i,7])                               # xDot
            #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + dt*(z_Ol[i,7] - 0.5*z_Ol[i,1]))                               # xDot
            @NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2]*z_Ol[i,3] + c_Vx[2]*z_Ol[i,1] + c_Vx[3]*z_Ol[i,7]) 
            @NLconstraint(mdl, z_Ol[i+1,4]  == z_Ol[i,4] + dt*(z_Ol[i,3]-dsdt[i]*c[i]))                                                                                 # ePsi
            @NLconstraint(mdl, z_Ol[i+1,5]  == z_Ol[i,5] + dt*(z_Ol[i,1]*sin(z_Ol[i,4])+z_Ol[i,2]*cos(z_Ol[i,4])))                                                      # eY
            @NLconstraint(mdl, z_Ol[i+1,6]  == z_Ol[i,6] + dt*dsdt[i]  )                                                                                                # s
        end
        # @NLconstraint(mdl, u_Ol[1,1]-uPrev[1,1] <= 0.05)
        # @NLconstraint(mdl, u_Ol[1,1]-uPrev[1,1] >= -0.2)
        # for i=1:N-1 # Constraints on u:
        #     @NLconstraint(mdl, u_Ol[i+1,1]-u_Ol[i,1] <= 0.05)
        #     @NLconstraint(mdl, u_Ol[i+1,1]-u_Ol[i,1] >= -0.2)
        # end

        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] <= 0.06)
        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] >= -0.06)
        for i=1:N-1 # Constraints on u:
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] <= 0.06)
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] >= -0.06)
        end

       
   
        # Cost functions

        # Derivative cost
        # ---------------------------------
        @NLexpression(mdl, derivCost, sum{QderivZ[j]*(sum{(z_Ol[i,j]-z_Ol[i+1,j])^2,i=1:N}),j=1:6} +
                                          QderivU[1]*((uPrev[1,1]-u_Ol[1,1])^2+sum{(u_Ol[i,1]-u_Ol[i+1,1])^2,i=1:N-1})+
                                          QderivU[2]*((uPrev[1,2]-u_Ol[1,2])^2+sum{(u_Ol[i,2]-u_Ol[i+1,2])^2,i=1:N-delay_df-1}))        

        # Control Input cost
        # ---------------------------------
        @NLexpression(mdl, controlCost, R[1]*sum{(u_Ol[i,1])^2,i=1:N-delay_a}+
                                        R[2]*sum{(u_Ol[i,2])^2,i=1:N-delay_df})

        # Lane cost (soft)
        # ---------------------------------
        @NLexpression(mdl, laneCost, Q_lane*sum{40.0*eps_lane[i]+300.0*eps_lane[i]^2 ,i=2:N+1})


        # Terminal Cost
        # ---------------------------------
        @NLexpression(mdl, terminalCost , sum{alpha[i]*statesCost[i], i=1:Nl*Np})

        # Slack cost (soft)
        # ---------------------------------
        #@NLexpression(mdl, slackCost, sum{50*eps_alpha[i]+500*eps_alpha[i]^2,i=1:6})

        # Slack cost on vx
        #----------------------------------
        @NLexpression(mdl, slackVx, (z_Ol[N+1,1] - sum{alpha[j]*selStates[j,1],j=1:Nl*Np})^2)

        # Slack cost on vy
        #----------------------------------
        @NLexpression(mdl, slackVy, (z_Ol[N+1,2] - sum{alpha[j]*selStates[j,2],j=1:Nl*Np})^2)

        # Slack cost on Psi dot
        #----------------------------------
        @NLexpression(mdl, slackPsidot, (z_Ol[N+1,3] - sum{alpha[j]*selStates[j,3],j=1:Nl*Np})^2)

        # Slack cost on ePsi
        #----------------------------------
        @NLexpression(mdl, slackEpsi, (z_Ol[N+1,4] - sum{alpha[j]*selStates[j,4],j=1:Nl*Np})^2)

        # Slack cost on ey
        #----------------------------------
        @NLexpression(mdl, slackEy, (z_Ol[N+1,5] - sum{alpha[j]*selStates[j,5],j=1:Nl*Np})^2)

        # Slack cost on s
        #----------------------------------
        @NLexpression(mdl, slackS, (z_Ol[N+1,6] - sum{alpha[j]*selStates[j,6],j=1:Nl*Np})^2)

        # Velocity Cost
        #----------------------------------
        #@NLexpression(mdl, velocityCost , Q_vel*sum{10.0*eps_vel[i]+100.0*eps_vel[i]^2 ,i=2:N+1})

        # Overall Cost function (objective of the minimization)
        # -----------------------------------------------------

        #@NLobjective(mdl, Min, derivCost + laneCost + controlCost + terminalCost )#+ slackCost)#+ velocityCost)

        @NLobjective(mdl, Min, derivCost + laneCost +  terminalCost + Q_slack[1]*slackVx + Q_slack[2]*slackVy + Q_slack[3]*slackPsidot + Q_slack[4]*slackEpsi + Q_slack[5]*slackEy + Q_slack[6]*slackS) #+ controlCost


        sol_stat=solve(mdl)
        println("Finished solve 1 convhull mpc: $sol_stat")
        sol_stat=solve(mdl)
        println("Finished solve 2 convhull mpc: $sol_stat")


        m.mdl = mdl
        m.z0 = z0
        m.coeff = coeff
        m.z_Ol = z_Ol
        m.u_Ol = u_Ol
        m.c_Vx = c_Vx
        m.c_Vy = c_Vy
        m.c_Psi = c_Psi
        m.uPrev = uPrev
        #m.eps_alpha=eps_alpha

        m.derivCost = derivCost
        m.controlCost = controlCost
        m.laneCost = laneCost
        m.terminalCost= terminalCost # terminal cost
        #m.velocityCost= velocityCost #velocity cost
        m.selStates   = selStates    # selected states
        m.statesCost  = statesCost   # cost of the selected states
        m.alpha       = alpha        # parameters of the convex hull

        m.slackVx     = slackVx
        m.slackVy     = slackVy
        m.slackPsidot = slackPsidot
        m.slackEpsi   = slackEpsi
        m.slackEy     = slackEy
        m.slackS      = slackS


        return m
    end
end



type MpcModel_test

    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    coeff::Array{JuMP.NonlinearParameter,1}
    selStates::Array{JuMP.NonlinearParameter,2}
    statesCost::Array{JuMP.NonlinearParameter,1}
    c_Vx::Array{JuMP.NonlinearParameter,1}
    c_Vy::Array{JuMP.NonlinearParameter,1}
    c_Psi::Array{JuMP.NonlinearParameter,1}
    uPrev::Array{JuMP.NonlinearParameter,2}


    eps_lane::Array{JuMP.Variable,1}
    #eps_alpha::Array{JuMP.Variable,1}
    #eps_vel::Array{JuMP.Variable,1}
    alpha::Array{JuMP.Variable,1}
    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}

    
    dsdt::Array{JuMP.NonlinearExpression,1}
    c::Array{JuMP.NonlinearExpression,1}

    derivCost::JuMP.NonlinearExpression
    controlCost::JuMP.NonlinearExpression
    laneCost::JuMP.NonlinearExpression
    terminalCost::JuMP.NonlinearExpression
    costPF::JuMP.NonlinearExpression
    #slackCost::JuMP.NonlinearExpression
    #velocityCost::JuMP.NonlinearExpression

    function MpcModel_test(mpcParams::MpcParams,mpcCoeff::MpcCoeff,modelParams::ModelParams,trackCoeff::TrackCoeff,selectedStates::SelectedStates)
                             

        m = new()

        #### Initialize parameters

        dt         = modelParams.dt                # time step
        L_a        = modelParams.l_A               # distance from CoM of the car to the front wheels
        L_b        = modelParams.l_B               # distance from CoM of the car to the rear wheels
        u_lb       = modelParams.u_lb              # lower bounds for the control inputs
        u_ub       = modelParams.u_ub              # upper bounds for the control inputs
        z_lb       = modelParams.z_lb              # lower bounds for the states
        z_ub       = modelParams.z_ub              # upper bounds for the states
        c0         = modelParams.c0


        ey_max      = trackCoeff.width/2           # bound for the state ey (distance from the center track). It is set as half of the width of the track for obvious reasons
        n_poly_curv = trackCoeff.nPolyCurvature    # polynomial degree for curvature approximation
        v_max       = 3                            # maximum allowed velocity

        v_ref      = mpcParams.vPathFollowing              # reference velocity for the path following 
        N          = mpcParams.N                           # Prediction horizon
        QderivZ    = mpcParams.QderivZ::Array{Float64,1}   # weights for the derivative cost on the states
        QderivU    = mpcParams.QderivU::Array{Float64,1}   # weights for the derivative cost on the control inputs
        R          = mpcParams.R::Array{Float64,1}         # weights on the control inputs
        Q          = mpcParams.Q::Array{Float64,1}         # weights on the states for path following
        Q_lane     = mpcParams.Q_lane::Float64             # weight on the soft constraint on the lane
        #Q_vel      = mpcParams.Q_vel::Float64              # weight on the soft constraint for the max velocity
        delay_df   = mpcParams.delay_df
        delay_a    = mpcParams.delay_a

      
        Np         = selectedStates.Np::Int64              # how many states to select
        Nl         = selectedStates.Nl::Int64              # how many previous laps to select

        acc_f           = 1.0

        # Create function-specific parameters
        z_Ref::Array{Float64,2}
        z_Ref       = cat(2,v_ref*ones(N+1,1),zeros(N+1,5))       # Reference trajectory: path following -> stay on line and keep constant velocity
        u_Ref       = zeros(N,2)


        mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.09))#,check_derivatives_for_naninf="yes"))#,linear_solver="ma57",print_user_options="yes"))

        @variable( mdl, z_Ol[1:(N+1),1:7])
        @variable( mdl, u_Ol[1:N,1:2])
        @variable( mdl, eps_lane[1:N+1] >= 0)   # eps for soft lane constraints
        @variable( mdl, alpha[1:Nl*Np] >= 0)    # coefficients of the convex hull
       # @variable( mdl, eps_alpha[1:6] >=0)     # eps for soft constraint on alpha
       #@variable( mdl, eps_vel[1:N+1]>=0)      # eps for soft constraint on velocity



        z_lb_6s = ones(mpcParams.N+1,1)*[0.1 -Inf -Inf -Inf -Inf -Inf -Inf]                      # lower bounds on states
        z_ub_6s = ones(mpcParams.N+1,1)*[3.5  Inf Inf  Inf  Inf  Inf Inf]                      # upper bounds
        u_lb_6s = ones(mpcParams.N,1) * [-1.0  -0.3]                                         # lower bounds on steering
        u_ub_6s = ones(mpcParams.N,1) * [2.0   0.3]                                         # upper bounds

        for i=1:2
            for j=1:N
                setlowerbound(u_Ol[j,i], u_lb_6s[j,i])
                setupperbound(u_Ol[j,i], u_ub_6s[j,i])
            end
        end
        for i=1:7
            for j=1:N+1
                setlowerbound(z_Ol[j,i], z_lb_6s[j,i])
                setupperbound(z_Ol[j,i], z_ub_6s[j,i])
            end
        end

        @NLparameter(mdl, z0[i=1:7] == 0)
        @NLparameter(mdl, coeff[i=1:n_poly_curv+1] == 0)
        @NLparameter(mdl, c_Vx[i=1:3]  == 0)
        @NLparameter(mdl, c_Vy[i=1:4]  == 0)
        @NLparameter(mdl, c_Psi[i=1:3] == 0)
        @NLparameter(mdl, uPrev[1:10,1:2] == 0)
        @NLparameter(mdl, selStates[1:Nl*Np,1:6] == 0)                                 # states from the previous trajectories selected in "convhullStates"
        @NLparameter(mdl, statesCost[1:Nl*Np] == 0)                                    # costs of the states selected in "convhullStates"

        
        # Conditions for first solve:
        setvalue(z0[1],1)
        setvalue(c_Vx[3],0.1)

        @NLconstraint(mdl, [i=1:7], z_Ol[1,i] == z0[i])

        @NLconstraint(mdl, [i=2:N+1], z_Ol[i,5] <= ey_max + eps_lane[i])
        @NLconstraint(mdl, [i=2:N+1], z_Ol[i,5] >= -ey_max - eps_lane[i])
        #@NLconstraint(mdl,[i = 1:(N+1)], z_Ol[i,4] <= v_max + eps_vel[i] )      # sof constraint on maximum velocity
        @NLconstraint(mdl, sum{alpha[i],i=1:Nl*Np} == 1)                        # constraint on the coefficients of the convex hull

        for i = 1:6
            @NLconstraint(mdl,z_Ol[N+1,i] == sum{alpha[j]*selStates[j,i],j=1:Nl*Np})  # terminal constraint
            #@NLconstraint(mdl,z_Ol[N+1,n] >= sum{alpha[j]*selStates[j,n],j=1:Nl*Np}-eps_alpha[n])  
            #@NLconstraint(mdl,z_Ol[N+1,n] <= sum{alpha[j]*selStates[j,n],j=1:Nl*Np}+eps_alpha[n])
        end  

        @NLexpression(mdl, c[i = 1:N], sum{coeff[j]*z_Ol[i,6]^(n_poly_curv-j+1),j=1:n_poly_curv} + coeff[n_poly_curv+1])
        @NLexpression(mdl, dsdt[i = 1:N], (z_Ol[i,1]*cos(z_Ol[i,4]) - z_Ol[i,2]*sin(z_Ol[i,4]))/(1-z_Ol[i,5]*c[i]))
        
        println("Initializing model...")

        # System dynamics
        for i=1:N
            if i<=delay_df
                @NLconstraint(mdl, z_Ol[i+1,2]  == z_Ol[i,2] + c_Vy[1]*z_Ol[i,2]/z_Ol[i,1] + c_Vy[2]*z_Ol[i,1]*z_Ol[i,3] + c_Vy[3]*z_Ol[i,3]/z_Ol[i,1] + c_Vy[4]*uPrev[delay_df+1-i,2]) # yDot
                @NLconstraint(mdl, z_Ol[i+1,3]  == z_Ol[i,3] + c_Psi[1]*z_Ol[i,3]/z_Ol[i,1] + c_Psi[2]*z_Ol[i,2]/z_Ol[i,1] + c_Psi[3]*uPrev[delay_df+1-i,2])                            # psiDot
            else
                @NLconstraint(mdl, z_Ol[i+1,2]  == z_Ol[i,2] + c_Vy[1]*z_Ol[i,2]/z_Ol[i,1] + c_Vy[2]*z_Ol[i,1]*z_Ol[i,3] + c_Vy[3]*z_Ol[i,3]/z_Ol[i,1] + c_Vy[4]*u_Ol[i-delay_df,2]) # yDot
                @NLconstraint(mdl, z_Ol[i+1,3]  == z_Ol[i,3] + c_Psi[1]*z_Ol[i,3]/z_Ol[i,1] + c_Psi[2]*z_Ol[i,2]/z_Ol[i,1] + c_Psi[3]*u_Ol[i-delay_df,2])                            # psiDot
            end
            if i<=delay_a
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + dt*(uPrev[delay_a+1-i,1] - 0.5*z_Ol[i,1]))
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*u_Ol[i,1])                              # xDot
                @NLconstraint(mdl, z_Ol[i+1,7]  == z_Ol[i,7] + dt*(uPrev[delay_a+1-i,1]-z_Ol[i,7])*acc_f)
            else
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*u_Ol[i,1])                              # xDot
                @NLconstraint(mdl, z_Ol[i+1,7]  == z_Ol[i,7] + dt*(u_Ol[i-delay_a,1]-z_Ol[i,7])*acc_f)
            end
            #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*z_Ol[i,7])                               # xDot
            #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + dt*(z_Ol[i,7] - 0.5*z_Ol[i,1]))                               # xDot
            @NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2]*z_Ol[i,3] + c_Vx[2]*z_Ol[i,1] + c_Vx[3]*z_Ol[i,7]) 
            @NLconstraint(mdl, z_Ol[i+1,4]  == z_Ol[i,4] + dt*(z_Ol[i,3]-dsdt[i]*c[i]))                                                                                 # ePsi
            @NLconstraint(mdl, z_Ol[i+1,5]  == z_Ol[i,5] + dt*(z_Ol[i,1]*sin(z_Ol[i,4])+z_Ol[i,2]*cos(z_Ol[i,4])))                                                      # eY
            @NLconstraint(mdl, z_Ol[i+1,6]  == z_Ol[i,6] + dt*dsdt[i]  )                                                                                                # s
        end
        @NLconstraint(mdl, u_Ol[1,1]-uPrev[1,1] <= 0.05)
        @NLconstraint(mdl, u_Ol[1,1]-uPrev[1,1] >= -0.2)
        for i=1:N-1 # Constraints on u:
            @NLconstraint(mdl, u_Ol[i+1,1]-u_Ol[i,1] <= 0.05)
            @NLconstraint(mdl, u_Ol[i+1,1]-u_Ol[i,1] >= -0.2)
        end

        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] <= 0.06)
        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] >= -0.06)
        for i=1:N-1 # Constraints on u:
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] <= 0.06)
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] >= -0.06)
        end

       
   
        # Cost functions

        # Derivative cost
        # ---------------------------------
        @NLexpression(mdl, derivCost, sum{QderivZ[j]*(sum{(z_Ol[i,j]-z_Ol[i+1,j])^2,i=1:N}),j=1:6} +
                                          QderivU[1]*((uPrev[1,1]-u_Ol[1,1])^2+sum{(u_Ol[i,1]-u_Ol[i+1,1])^2,i=1:N-delay_a-1})+
                                          QderivU[2]*((uPrev[1,2]-u_Ol[1,2])^2+sum{(u_Ol[i,2]-u_Ol[i+1,2])^2,i=1:N-delay_df-1}))        

        # Control Input cost
        # ---------------------------------
        @NLexpression(mdl, controlCost, R[1]*sum{(u_Ol[i,1])^2,i=1:N-delay_a}+
                                        R[2]*sum{(u_Ol[i,2])^2,i=1:N-delay_df})

        # Lane cost (soft)
        # ---------------------------------
        @NLexpression(mdl, laneCost, Q_lane*sum{10.0*eps_lane[i]+100.0*eps_lane[i]^2 ,i=2:N+1})

        # Terminal Cost
        # ---------------------------------
        @NLexpression(mdl, terminalCost , sum{alpha[i]*statesCost[i], i=1:Nl*Np})

        # Slack cost (soft)
        # ---------------------------------
        #@NLexpression(mdl, slackCost, sum{50*eps_alpha[i]+500*eps_alpha[i]^2,i=1:6})

        # Velocity Cost
        #----------------------------------
        #@NLexpression(mdl, velocityCost , Q_vel*sum{10.0*eps_vel[i]+100.0*eps_vel[i]^2 ,i=2:N+1})

        # Path Following Cost
        #----------------------------------

        @NLexpression(mdl, costPF, 0.5*sum{Q[i]*sum{(z_Ol[j,i]-z_Ref[j,i])^2,j=2:N+1},i=1:6})    # Follow trajectory


        # Overall Cost function (objective of the minimization)
        # -----------------------------------------------------

        @NLobjective(mdl, Min, derivCost + laneCost + controlCost + terminalCost )#+ slackCost)#+ velocityCost)
        #@NLobjective(mdl, Min, derivCost + controlCost + costPF + terminalCost)


        sol_stat=solve(mdl)
        println("Finished solve 1: $sol_stat")
        sol_stat=solve(mdl)
        println("Finished solve 2: $sol_stat")


        m.mdl = mdl
        m.z0 = z0
        m.coeff = coeff
        m.z_Ol = z_Ol
        m.u_Ol = u_Ol
        m.c_Vx = c_Vx
        m.c_Vy = c_Vy
        m.c_Psi = c_Psi
        m.uPrev = uPrev
        #m.eps_alpha=eps_alpha

        m.derivCost = derivCost
        m.controlCost = controlCost
        m.costPF = costPF
        #m.laneCost = laneCost
        m.terminalCost= terminalCost # terminal cost
        #m.velocityCost= velocityCost #velocity cost
        m.selStates   = selStates    # selected states
        m.statesCost  = statesCost   # cost of the selected states
        m.alpha       = alpha        # parameters of the convex hull


        return m
    end
end

type MpcModel_obstacle

    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    coeff::Array{JuMP.NonlinearParameter,1}
    selStates::Array{JuMP.NonlinearParameter,2}
    statesCost::Array{JuMP.NonlinearParameter,1}
    c_Vx::Array{JuMP.NonlinearParameter,1}
    c_Vy::Array{JuMP.NonlinearParameter,1}
    c_Psi::Array{JuMP.NonlinearParameter,1}
    uPrev::Array{JuMP.NonlinearParameter,2}
    Q_obs::Array{JuMP.NonlinearParameter,1}
    obs::Array{JuMP.NonlinearParameter,2}


    eps_lane::Array{JuMP.Variable,1}
    #eps_vel::Array{JuMP.Variable,1}
    alpha::Array{JuMP.Variable,1}
    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}

    
    dsdt::Array{JuMP.NonlinearExpression,1}
    c::Array{JuMP.NonlinearExpression,1}

    derivCost::JuMP.NonlinearExpression
    controlCost::JuMP.NonlinearExpression
    laneCost::JuMP.NonlinearExpression
    terminalCost::JuMP.NonlinearExpression
    slackVx::JuMP.NonlinearExpression
    slackVy::JuMP.NonlinearExpression
    slackPsidot::JuMP.NonlinearExpression
    slackEpsi::JuMP.NonlinearExpression
    slackEy::JuMP.NonlinearExpression
    slackS::JuMP.NonlinearExpression
    obstacleSlackCost::JuMP.NonlinearExpression

    #velocityCost::JuMP.NonlinearExpression

    function MpcModel_obstacle(mpcParams::MpcParams,mpcCoeff::MpcCoeff,modelParams::ModelParams,trackCoeff::TrackCoeff,selectedStates::SelectedStates,obstacle::Obstacle)
                             

        m = new()

        #### Initialize parameters

        dt         = modelParams.dt                # time step
        L_a        = modelParams.l_A               # distance from CoM of the car to the front wheels
        L_b        = modelParams.l_B               # distance from CoM of the car to the rear wheels
        u_lb       = modelParams.u_lb              # lower bounds for the control inputs
        u_ub       = modelParams.u_ub              # upper bounds for the control inputs
        z_lb       = modelParams.z_lb              # lower bounds for the states
        z_ub       = modelParams.z_ub              # upper bounds for the states
        c0         = modelParams.c0


        ey_max      = trackCoeff.width/2           # bound for the state ey (distance from the center track). It is set as half of the width of the track for obvious reasons
        n_poly_curv = trackCoeff.nPolyCurvature    # polynomial degree for curvature approximation
        v_max       = 3                            # maximum allowed velocity

        v_ref      = mpcParams.vPathFollowing              # reference velocity for the path following 
        N          = mpcParams.N                           # Prediction horizon
        QderivZ    = mpcParams.QderivZ::Array{Float64,1}   # weights for the derivative cost on the states
        QderivU    = mpcParams.QderivU::Array{Float64,1}   # weights for the derivative cost on the control inputs
        R          = mpcParams.R::Array{Float64,1}         # weights on the control inputs
        Q          = mpcParams.Q::Array{Float64,1}         # weights on the states for path following
        Q_lane     = mpcParams.Q_lane::Float64             # weight on the soft constraint on the lane
        #Q_vel      = mpcParams.Q_vel::Float64              # weight on the soft constraint for the max velocity
        delay_df   = mpcParams.delay_df
        delay_a    = mpcParams.delay_a
        Q_slack    = mpcParams.Q_slack

        r_s        = obstacle.r_s
        r_ey       = obstacle.r_ey

      
        Np         = selectedStates.Np::Int64              # how many states to select
        Nl         = selectedStates.Nl::Int64              # how many previous laps to select

        acc_f           = 1.0


        mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.09))#,check_derivatives_for_naninf="yes"))#,linear_solver="ma57",print_user_options="yes"))

        @variable( mdl, z_Ol[1:(N+1),1:7])
        @variable( mdl, u_Ol[1:N,1:2])
        @variable( mdl, eps_lane[1:N+1] >= 0)   # eps for soft lane constraints
        @variable( mdl, alpha[1:Nl*Np] >= 0)    # coefficients of the convex hull
       # @variable( mdl, eps_alpha[1:6] >=0)     # eps for soft constraint on alpha
       #@variable( mdl, eps_vel[1:N+1]>=0)      # eps for soft constraint on velocity



        z_lb_6s = ones(mpcParams.N+1,1)*[0.1 -Inf -Inf -Inf -Inf -Inf -Inf]                      # lower bounds on states
        z_ub_6s = ones(mpcParams.N+1,1)*[3.5  Inf Inf  Inf  Inf  Inf Inf]                      # upper bounds
        u_lb_6s = ones(mpcParams.N,1) * [-1.0  -0.3]                                         # lower bounds on steering
        u_ub_6s = ones(mpcParams.N,1) * [2.0   0.3]                                         # upper bounds

        for i=1:2
            for j=1:N
                setlowerbound(u_Ol[j,i], u_lb_6s[j,i])
                setupperbound(u_Ol[j,i], u_ub_6s[j,i])
            end
        end
        for i=1:7
            for j=1:N+1
                setlowerbound(z_Ol[j,i], z_lb_6s[j,i])
                setupperbound(z_Ol[j,i], z_ub_6s[j,i])
            end
        end

        @NLparameter(mdl, z0[i=1:7] == 0)
        @NLparameter(mdl, coeff[i=1:n_poly_curv+1] == 0)
        @NLparameter(mdl, c_Vx[i=1:3]  == 0)
        @NLparameter(mdl, c_Vy[i=1:4]  == 0)
        @NLparameter(mdl, c_Psi[i=1:3] == 0)
        @NLparameter(mdl, uPrev[1:10,1:2] == 0)
        @NLparameter(mdl, selStates[1:Nl*Np,1:6] == 0)                                 # states from the previous trajectories selected in "convhullStates"
        @NLparameter(mdl, statesCost[1:Nl*Np] == 0)                                    # costs of the states selected in "convhullStates"
        @NLparameter(mdl, Q_obs[i=1:Nl*Np] == mpcParams.Q_obs[i])                     # weight used to exclude some states from the convex hull
        @NLparameter(mdl, obs[j=1:N+1,i=1:3] == 0)                                    # nearest obstacle to avoid

        
        # Conditions for first solve:
        setvalue(z0[1],1)
        setvalue(c_Vx[3],0.1)

        @NLconstraint(mdl, [i=1:7], z_Ol[1,i] == z0[i])

        @NLconstraint(mdl, [i=2:N+1], z_Ol[i,5] <= ey_max + eps_lane[i])
        @NLconstraint(mdl, [i=2:N+1], z_Ol[i,5] >= -ey_max - eps_lane[i])
        #@NLconstraint(mdl,[i = 1:(N+1)], z_Ol[i,4] <= v_max + eps_vel[i] )      # sof constraint on maximum velocity
        @NLconstraint(mdl, sum{alpha[i],i=1:Nl*Np} == 1)                        # constraint on the coefficients of the convex hull

        #for n = 1:6
            #@NLconstraint(mdl,z_Ol[N+1,n] == sum{alpha[j]*selStates[j,n],j=1:Nl*Np})  # terminal constraint
            #@NLconstraint(mdl,z_Ol[N+1,n] >= sum{alpha[j]*selStates[j,n],j=1:Nl*Np}-eps_alpha[n])  
            #@NLconstraint(mdl,z_Ol[N+1,n] <= sum{alpha[j]*selStates[j,n],j=1:Nl*Np}+eps_alpha[n])
        #end  

        @NLexpression(mdl, c[i = 1:N], sum{coeff[j]*z_Ol[i,6]^(n_poly_curv-j+1),j=1:n_poly_curv} + coeff[n_poly_curv+1])
        @NLexpression(mdl, dsdt[i = 1:N], (z_Ol[i,1]*cos(z_Ol[i,4]) - z_Ol[i,2]*sin(z_Ol[i,4]))/(1-z_Ol[i,5]*c[i]))
        
        println("Initializing model...")

        # System dynamics
        for i=1:N
            if i<=delay_df
                @NLconstraint(mdl, z_Ol[i+1,2]  == z_Ol[i,2] + c_Vy[1]*z_Ol[i,2]/z_Ol[i,1] + c_Vy[2]*z_Ol[i,1]*z_Ol[i,3] + c_Vy[3]*z_Ol[i,3]/z_Ol[i,1] + c_Vy[4]*uPrev[delay_df+1-i,2]) # yDot
                @NLconstraint(mdl, z_Ol[i+1,3]  == z_Ol[i,3] + c_Psi[1]*z_Ol[i,3]/z_Ol[i,1] + c_Psi[2]*z_Ol[i,2]/z_Ol[i,1] + c_Psi[3]*uPrev[delay_df+1-i,2])                            # psiDot
            else
                @NLconstraint(mdl, z_Ol[i+1,2]  == z_Ol[i,2] + c_Vy[1]*z_Ol[i,2]/z_Ol[i,1] + c_Vy[2]*z_Ol[i,1]*z_Ol[i,3] + c_Vy[3]*z_Ol[i,3]/z_Ol[i,1] + c_Vy[4]*u_Ol[i-delay_df,2]) # yDot
                @NLconstraint(mdl, z_Ol[i+1,3]  == z_Ol[i,3] + c_Psi[1]*z_Ol[i,3]/z_Ol[i,1] + c_Psi[2]*z_Ol[i,2]/z_Ol[i,1] + c_Psi[3]*u_Ol[i-delay_df,2])                            # psiDot
            end
            if i<=delay_a
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + dt*(uPrev[delay_a+1-i,1] - 0.5*z_Ol[i,1]))
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*u_Ol[i,1])                              # xDot
                @NLconstraint(mdl, z_Ol[i+1,7]  == z_Ol[i,7] + dt*(uPrev[delay_a+1-i,1]-z_Ol[i,7])*acc_f)
            else
                #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*u_Ol[i,1])                              # xDot
                @NLconstraint(mdl, z_Ol[i+1,7]  == z_Ol[i,7] + dt*(u_Ol[i-delay_a,1]-z_Ol[i,7])*acc_f)
            end
            #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2] + c_Vx[2]*z_Ol[i,3] + c_Vx[3]*z_Ol[i,1] + c_Vx[4]*z_Ol[i,7])                               # xDot
            #@NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + dt*(z_Ol[i,7] - 0.5*z_Ol[i,1]))                               # xDot
            @NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + c_Vx[1]*z_Ol[i,2]*z_Ol[i,3] + c_Vx[2]*z_Ol[i,1] + c_Vx[3]*z_Ol[i,7]) 
            @NLconstraint(mdl, z_Ol[i+1,4]  == z_Ol[i,4] + dt*(z_Ol[i,3]-dsdt[i]*c[i]))                                                                                 # ePsi
            @NLconstraint(mdl, z_Ol[i+1,5]  == z_Ol[i,5] + dt*(z_Ol[i,1]*sin(z_Ol[i,4])+z_Ol[i,2]*cos(z_Ol[i,4])))                                                      # eY
            @NLconstraint(mdl, z_Ol[i+1,6]  == z_Ol[i,6] + dt*dsdt[i]  )                                                                                                # s
        end
        # @NLconstraint(mdl, u_Ol[1,1]-uPrev[1,1] <= 0.05)
        # @NLconstraint(mdl, u_Ol[1,1]-uPrev[1,1] >= -0.2)
        # for i=1:N-1 # Constraints on u:
        #     @NLconstraint(mdl, u_Ol[i+1,1]-u_Ol[i,1] <= 0.05)
        #     @NLconstraint(mdl, u_Ol[i+1,1]-u_Ol[i,1] >= -0.2)
        # end

        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] <= 0.06)
        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] >= -0.06)
        for i=1:N-1 # Constraints on u:
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] <= 0.06)
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] >= -0.06)
        end

       
   
        # Cost functions

        # Derivative cost
        # ---------------------------------
        @NLexpression(mdl, derivCost, sum{QderivZ[j]*(sum{(z_Ol[i,j]-z_Ol[i+1,j])^2,i=1:N}),j=1:6} +
                                          QderivU[1]*((uPrev[1,1]-u_Ol[1,1])^2+sum{(u_Ol[i,1]-u_Ol[i+1,1])^2,i=1:N-delay_a-1})+
                                          QderivU[2]*((uPrev[1,2]-u_Ol[1,2])^2+sum{(u_Ol[i,2]-u_Ol[i+1,2])^2,i=1:N-delay_df-1}))        

        # Control Input cost
        # ---------------------------------
        @NLexpression(mdl, controlCost, R[1]*sum{(u_Ol[i,1])^2,i=1:N-delay_a}+
                                        R[2]*sum{(u_Ol[i,2])^2,i=1:N-delay_df})

        # Lane cost (soft)
        # ---------------------------------
        @NLexpression(mdl, laneCost, Q_lane*sum{10.0*eps_lane[i]+100.0*eps_lane[i]^2 ,i=2:N+1})

        # Terminal Cost
        # ---------------------------------
        @NLexpression(mdl, terminalCost , sum{Q_obs[i]*alpha[i]*statesCost[i], i=1:Nl*Np})

        # Slack cost (soft)
        # ---------------------------------
        #@NLexpression(mdl, slackCost, sum{50*eps_alpha[i]+500*eps_alpha[i]^2,i=1:6})

        # Slack cost on vx
        #----------------------------------
        @NLexpression(mdl, slackVx, (z_Ol[N+1,1] - sum{alpha[j]*selStates[j,1],j=1:Nl*Np})^2)

        # Slack cost on vy
        #----------------------------------
        @NLexpression(mdl, slackVy, (z_Ol[N+1,2] - sum{alpha[j]*selStates[j,2],j=1:Nl*Np})^2)

        # Slack cost on Psi dot
        #----------------------------------
        @NLexpression(mdl, slackPsidot, (z_Ol[N+1,3] - sum{alpha[j]*selStates[j,3],j=1:Nl*Np})^2)

        # Slack cost on ePsi
        #----------------------------------
        @NLexpression(mdl, slackEpsi, (z_Ol[N+1,4] - sum{alpha[j]*selStates[j,4],j=1:Nl*Np})^2)

        # Slack cost on ey
        #----------------------------------
        @NLexpression(mdl, slackEy, (z_Ol[N+1,5] - sum{alpha[j]*selStates[j,5],j=1:Nl*Np})^2)

        # Slack cost on s
        #----------------------------------
        @NLexpression(mdl, slackS, (z_Ol[N+1,6] - sum{alpha[j]*selStates[j,6],j=1:Nl*Np})^2)

        # Soft Constraint on the Obstacle
        # --------------------------------
        @NLexpression(mdl, obstacleSlackCost, 0.1*sum{-log(((z_Ol[i,6]-obs[i,1])/r_s)^2 + ((z_Ol[i,5]-obs[i,2])/r_ey)^2 -1),i=1:N+1})


        # Velocity Cost
        #----------------------------------
        #@NLexpression(mdl, velocityCost , Q_vel*sum{10.0*eps_vel[i]+100.0*eps_vel[i]^2 ,i=2:N+1})

        # Overall Cost function (objective of the minimization)
        # -----------------------------------------------------

        #@NLobjective(mdl, Min, derivCost + laneCost + controlCost + terminalCost )#+ slackCost)#+ velocityCost)

        @NLobjective(mdl, Min, derivCost + laneCost + controlCost + terminalCost + Q_slack[1]*slackVx + Q_slack[2]*slackVy + Q_slack[3]*slackPsidot + Q_slack[4]*slackEpsi + Q_slack[5]*slackEy + Q_slack[6]*slackS + obstacleSlackCost)



        sol_stat=solve(mdl)
        println("Finished solve 1 convhull mpc: $sol_stat")
        sol_stat=solve(mdl)
        println("Finished solve 2 convhull mpc: $sol_stat")


        m.mdl = mdl
        m.z0 = z0
        m.coeff = coeff
        m.z_Ol = z_Ol
        m.u_Ol = u_Ol
        m.c_Vx = c_Vx
        m.c_Vy = c_Vy
        m.c_Psi = c_Psi
        m.uPrev = uPrev
        m.Q_obs       = Q_obs       # weigth to exclude states from optimization problem
        m.obs         = obs         # obstacle to avoid
        #m.eps_alpha=eps_alpha

        m.derivCost = derivCost
        m.controlCost = controlCost
        m.laneCost = laneCost
        m.terminalCost= terminalCost # terminal cost
        #m.velocityCost= velocityCost #velocity cost
        m.selStates   = selStates    # selected states
        m.statesCost  = statesCost   # cost of the selected states
        m.alpha       = alpha        # parameters of the convex hull

        m.obstacleSlackCost=obstacleSlackCost

        m.slackVx     = slackVx
        m.slackVy     = slackVy
        m.slackPsidot = slackPsidot
        m.slackEpsi   = slackEpsi
        m.slackEy     = slackEy
        m.slackS      = slackS


        return m
    end
end

