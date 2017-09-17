# Run this file using Julia 0.4.7

type MpcModel
    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    coeff::Array{JuMP.NonlinearParameter,1}

    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}

    derivCost::JuMP.NonlinearExpression
    costZ::JuMP.NonlinearExpression
    controlCost::JuMP.NonlinearExpression

    uPrev::Array{JuMP.NonlinearParameter,2}

    function MpcModel(mpcParams::MpcParams, modelParams::ModelParams, trackCoeff::TrackCoeff)
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

        N           = mpcParams.N
        Q           = mpcParams.Q
        R           = mpcParams.R
        QderivZ     = mpcParams.QderivZ::Array{Float64,1}
        QderivU     = mpcParams.QderivU::Array{Float64,1}
        delay_df    = mpcParams.delay_df::Int64
        delay_a     = mpcParams.delay_a::Int64

        v_ref       = mpcParams.vPathFollowing

        acc_f       = 1.0

        n_poly_curv = trackCoeff.nPolyCurvature         # polynomial degree of curvature approximation

        # Create function-specific parameters
        z_Ref::Array{Float64,2}
        z_Ref       = cat(2,zeros(N+1,3),v_ref*ones(N+1,1))       # Reference trajectory: path following -> stay on line and keep constant velocity
        u_Ref       = zeros(N,2)

        # Create Model
        mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.07))#,linear_solver="ma57",print_user_options="yes"))

        # Create variables (these are going to be optimized)
        @variable( mdl, z_Ol[1:(N+1),1:5], start = 0)          # z = s, ey, epsi, v
        @variable( mdl, u_Ol[1:N,1:2], start = 0)

        # Set bounds
        z_lb_4s = ones(mpcParams.N+1,1)*[-Inf -Inf -Inf -0.5]                  # lower bounds on states
        z_ub_4s = ones(mpcParams.N+1,1)*[ Inf  Inf  Inf  1.5]                  # upper bounds
        
        # Steering range: 33 <= steering angle <= 150 (values are PWM signals).  This corresponds to 0.749 >= steering angle >= -0.592 in radians

        u_lb_4s = ones(mpcParams.N,1) * [0.0  -0.592]                            # lower bounds on acceleration and steering
        u_ub_4s = ones(mpcParams.N,1) * [1.2   0.749]                            # upper bounds on acceleration and steering

        for i=1:2
            for j=1:N
                setlowerbound(u_Ol[j,i], u_lb_4s[j,i])
                setupperbound(u_Ol[j,i], u_ub_4s[j,i])
            end
        end

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
                @NLconstraint(mdl, z_Ol[i+1,5] == z_Ol[i,5] + dt*(uPrev[delay_a+1-i,1] - z_Ol[i,5])*acc_f)  # v
            else
                @NLconstraint(mdl, z_Ol[i+1,5] == z_Ol[i,5] + dt*(u_Ol[i-delay_a,1] - z_Ol[i,5])*acc_f)     # v
            end  


            @NLexpression(mdl, dsdt[i], z_Ol[i,4]*cos(z_Ol[i,3]+bta[i])/(1-z_Ol[i,2]*c[i]))
            @NLconstraint(mdl, z_Ol[i+1,1] == z_Ol[i,1] + dt*dsdt[i]  )                                                # s/acc)
            @NLconstraint(mdl, z_Ol[i+1,2] == z_Ol[i,2] + dt*z_Ol[i,4]*sin(z_Ol[i,3]+bta[i])  )                        # ey
            @NLconstraint(mdl, z_Ol[i+1,3] == z_Ol[i,3] + dt*(z_Ol[i,4]/L_a*sin(bta[i])-dsdt[i]*c[i])  )               # epsi
            @NLconstraint(mdl, z_Ol[i+1,4] == z_Ol[i,4] + dt*(z_Ol[i,5] - 0.5*z_Ol[i,4]))                              # v
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
            setvalue(z_Ol[i,:],[(i-1)*dt*v_ref, 0, 0, v_ref, 0])
        end
        for i=1:N
            setvalue(u_Ol[i,:],[0.15, 0])
        end
        # First solve
        sol_stat=solve(mdl)
        println("Finished solve 1: $sol_stat")
        sol_stat=solve(mdl)
        println("Finished solve 2: $sol_stat")
        
        m.mdl = mdl
        m.z0 = z0
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

# why does the solution change when I start the simulation twice, totally independently? Is not everything initialized equally?
# why do I get a failed restoration phase even though there are *almost* no constraints and the solution should be super clear?

# does it actually make sense to use the first (initial) state as a @variable and combine it with a constraint to fix it to z0 ?
# or better use only a parameter for z0? -> Found out that solutions are different if I set constraints for first state or not!?
