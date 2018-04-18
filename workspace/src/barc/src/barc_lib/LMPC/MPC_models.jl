type MpcModel_pF
    # Fields here provides a channel for JuMP model to communicate with data outside of it
    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    c::Array{JuMP.NonlinearParameter,1}
    z_Ref::Array{JuMP.NonlinearParameter,2}

    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}

    derivCost::JuMP.NonlinearExpression     # cost for state and input change
    costZ::JuMP.NonlinearExpression         # cost to the reference state
    controlCost::JuMP.NonlinearExpression   # soft constraints cost

    uPrev::Array{JuMP.NonlinearParameter,2}

    function MpcModel_pF(mpcParams_pF::MpcParams,modelParams::ModelParams)
        m = new()
        # Model parameters
        dt          = modelParams.dt
        L_a         = modelParams.l_A
        L_b         = modelParams.l_B
        u_lb        = mpcParams_pF.u_lb
        u_ub        = mpcParams_pF.u_ub
        z_lb        = mpcParams_pF.z_lb
        z_ub        = mpcParams_pF.z_ub
        c_f         = modelParams.c_f   # motor drag coefficient
        # MPC prameters
        N           = mpcParams_pF.N
        Q           = mpcParams_pF.Q
        R           = mpcParams_pF.R
        QderivZ     = mpcParams_pF.QderivZ
        QderivU     = mpcParams_pF.QderivU
        # Problem specific parameters
        v_ref       = mpcParams_pF.vPathFollowing

        # Create Model
        mdl = Model(solver = IpoptSolver(print_level=0,linear_solver="ma27")) #,max_cpu_time=0.09))#,linear_solver="ma57",print_user_options="yes"))

        # Create variables (these are going to be optimized)
        @variable( mdl, z_Ol[1:(N+1),1:4], start = 0)         # z = s, ey, epsi, v
        @variable( mdl, u_Ol[1:N,1:2], start = 0)

        # Set bounds
        z_lb_4s = ones(N+1,1)*z_lb # lower bounds on states: hard constraints
        z_ub_4s = ones(N+1,1)*z_ub # upper bounds on states: hard constraints
        u_lb_4s = ones(N,1)  *u_lb # lower bounds on steering: hard constraints
        u_ub_4s = ones(N,1)  *u_ub # upper bounds on steering: hard constraints

        for i=1:2
            for j=1:N
                setlowerbound(u_Ol[j,i], u_lb_4s[j,i])
                setupperbound(u_Ol[j,i], u_ub_4s[j,i])
            end
        end
        # path follower is really conservative controller regarding speed constraint,
        # so there is actully no need to set more constaints for states

        # Nonlinear parameters initialization
        @NLparameter(mdl, z_Ref[1:N+1,1:4]==0); setvalue(z_Ref,hcat(zeros(N+1,3),v_ref*ones(N+1,1)))
        @NLparameter(mdl, z0[i=1:4]==0);        setvalue(z0[4],v_ref) # initial speed for first initial solution
        @NLparameter(mdl, uPrev[1:N,1:2]==0)
        @NLparameter(mdl, zPrev[1:N+1,1:4]==0)
        @NLparameter(mdl, c[1:N+1]==0)

        # System dynamics
        @NLconstraint(mdl, [i=1:4], z_Ol[1,i] == z0[i])         # initial condition
        for i=1:N
            @NLexpression(mdl, bta[i],atan( L_a / (L_a + L_b) * tan(u_Ol[i,2])))
            @NLexpression(mdl, dsdt[i], z_Ol[i,4]*cos(z_Ol[i,3]+bta[i])/(1-z_Ol[i,2]*c[i]))
            @NLconstraint(mdl, z_Ol[i+1,1] == z_Ol[i,1] + dt*dsdt[i]  )                                     # s
            @NLconstraint(mdl, z_Ol[i+1,2] == z_Ol[i,2] + dt*z_Ol[i,4]*sin(z_Ol[i,3]+bta[i])  )             # ey
            @NLconstraint(mdl, z_Ol[i+1,3] == z_Ol[i,3] + dt*(z_Ol[i,4]/L_a*sin(bta[i])-dsdt[i]*c[i])  )    # epsi
            @NLconstraint(mdl, z_Ol[i+1,4] == z_Ol[i,4] + dt*(u_Ol[i,1] - c_f*z_Ol[i,4]))                   # v
        end

        ###### ------ Cost definitions ------ ######
        # Derivative cost
        @NLexpression(mdl, derivCost, sum{QderivZ[j]*(sum{(z_Ol[i,j]-z_Ol[i+1,j])^2,i=1:N}),j=1:4} +
                                      sum{QderivU[j]*(sum{(u_Ol[i,j]-u_Ol[i+1,j])^2,i=1:N-1}+(uPrev[1,j]-u_Ol[1,j])^2),j=1:2})
        # Control Input cost
        @NLexpression(mdl, controlCost, 0.5*sum{R[1]*sum{(u_Ol[i,1])^2,i=1:N},j=1:2})
        # State cost
        @NLexpression(mdl, costZ, 0.5*sum{Q[j]*sum{(z_Ol[i,j]-z_Ref[i,j])^2,i=2:N+1},j=1:4})

        # Objective function
        @NLobjective(mdl, Min, costZ + derivCost + controlCost)

        m.mdl=mdl
        m.z0=z0; m.z_Ref=z_Ref; m.c=c;
        m.z_Ol=z_Ol; m.u_Ol = u_Ol
        m.uPrev=uPrev
        m.derivCost=derivCost; m.costZ=costZ; m.controlCost=controlCost
        return m
    end
end