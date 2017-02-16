type MpcModel
    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    coeff::Array{JuMP.NonlinearParameter,1}
    coeffTermConst::Array{JuMP.NonlinearParameter,3}
    coeffTermCost::Array{JuMP.NonlinearParameter,2}
    c::Array{JuMP.NonlinearExpression,1}
    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}
    ParInt::JuMP.Variable
    eps::Array{JuMP.Variable,1}
    laneCost::JuMP.NonlinearExpression
    constZTerm::JuMP.NonlinearExpression
    costZTerm::JuMP.NonlinearExpression
    derivCost::JuMP.NonlinearExpression
    controlCost::JuMP.NonlinearExpression
    modelErrorCost::JuMP.NonlinearExpression

    uPrev::Array{JuMP.NonlinearParameter,2}
    rhoPrev::Array{JuMP.NonlinearParameter,1}
 
   function MpcModel(mpcParams::MpcParams,mpcCoeff::MpcCoeff,modelParams::ModelParams,trackCoeff::TrackCoeff,posInfo::PosInfo)
        m = new()
        dt   = modelParams.dt
        L_a  = modelParams.l_A 
        L_b  = modelParams.l_B 
        if L_a == L_b
            Lref = L_a
        else
            error("The MPC equations assume that L_a == L_b, which is not the case for your provided parameters!")
        end
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
        Q_modelError    = mpcParams.Q_modelError::Float64
        Q_term_cost     = mpcParams.Q_term_cost::Float64
        delay_df        = mpcParams.delay_df
        delay_a         = mpcParams.delay_a

        acc_f           = 1.0 #low pass filter parameter

        n_poly_curv = trackCoeff.nPolyCurvature         # polynomial degree of curvature approximation
        
        mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.09))

        @variable( mdl, z_Ol[1:(N+1),1:7])  # [s,ey,epsi,v,rho,epsiRef]
        @variable( mdl, u_Ol[1:N,1:3])      # [a,deltaF,phi]
        @variable( mdl, 0 <= ParInt <= 1)   # for convex combination of ss trajectories
        @variable(mdl, eps[1:7] >=0)        # for soft constraints

        z_lb_4s = ones(mpcParams.N+1,1)*[-Inf -Inf -Inf -Inf -Inf -Inf -Inf]                      # lower bounds on states
        z_ub_4s = ones(mpcParams.N+1,1)*[Inf  Inf Inf  Inf  Inf Inf Inf]                      # upper bounds
        u_lb_4s = ones(mpcParams.N,1) * [-0.75 -0.3 -100.0]                                         # lower bounds on inputs
        u_ub_4s = ones(mpcParams.N,1) * [1.5 0.3 100.0]                                         # upper bounds

        for i=1:3
            for j=1:N
                setlowerbound(u_Ol[j,i], u_lb_4s[j,i])
                setupperbound(u_Ol[j,i], u_ub_4s[j,i])
            end
        end
        for i=1:7
            for j=1:N+1
                setlowerbound(z_Ol[j,i], z_lb_4s[j,i])
                setupperbound(z_Ol[j,i], z_ub_4s[j,i])
            end
        end

        @NLparameter(mdl, z0[i=1:7] == 0)
        @NLparameter(mdl, coeff[i=1:n_poly_curv+1] == 0)
        @NLparameter(mdl, coeffTermConst[i=1:order+1,j=1:2,k=1:5] == 0)
        @NLparameter(mdl, coeffTermCost[i=1:order+1,j=1:2] == 0)
        @NLparameter(mdl, uPrev[1:10,1:2] == 0)
        @NLparameter(mdl, rhoPrev[1:10] == 0)
        # curvature polynomial relation 
        @NLexpression(mdl, c[i = 1:N], sum{coeff[j]*z_Ol[i,1]^(n_poly_curv-j+1),j=1:n_poly_curv} + coeff[n_poly_curv+1])

        # Conditions for first solve:
        setvalue(z0[1],1)   #m: Necessary? FIXME

        # Initial state constraint
        # ---------------------------------        
        @NLconstraint(mdl, [i=1:7], z_Ol[1,i] == z0[i])
   
        # State constraints
        # ---------------------------------  
        #@NLconstraint(mdl, [i=1:N+1], 8 <= z_Ol[i,5]  <= 15 )
        @NLconstraint(mdl, [i=1:N+1], z_Ol[i,2] <= ey_max + eps[6])
        @NLconstraint(mdl, [i=1:N+1], z_Ol[i,2] >= -ey_max - eps[7])

        println("Initializing model...")

        # System dynamics
        # z_Ol[1] = s
        # z_Ol[2] = ey
        # z_Ol[3] = epsi
        # z_Ol[4] = v
        # z_Ol[5] = rho
        # z_Ol[6] = epsi_ref
        # z_Ol[7] = filter state

        
        # Implementation of time delay 
        # ---------------------------------  
        # We assume a time delay of delay_df for the steering and delay_a for the acceleration
        # (time steps it takes until the inputs are actually executed by the system)
        # This is handled by simulating the first delay_df equations in the MPC with the already calculated steering inputs from the previous delay_df steps (saved in uPrev). The real decision for the next input is made in step delay_df+1 and applied to the system (same happens for delay_a)
      for i=1:N
            if i<=delay_df
                @NLexpression(mdl, bta[i],  atan( 0.5 * tan( uPrev[delay_df+1-i,2] ) ) )
              # @NLconstraint(mdl, z_Ol[i,5] == rhoPrev[delay_df+1-i])            # rho_est
            else
                @NLexpression(mdl, bta[i],  atan( 0.5 * tan( u_Ol[i-delay_df,2] ) ) )
            end
            if i<=delay_a
                @NLconstraint(mdl, z_Ol[i+1,7] == z_Ol[i,7] + dt*(uPrev[delay_a+1-i,1] - z_Ol[i,7])*acc_f)  
            else
                @NLconstraint(mdl, z_Ol[i+1,7] == z_Ol[i,7] + dt*(u_Ol[i-delay_a,1] - z_Ol[i,7])*acc_f)     
            end

            @NLexpression(mdl, dsdt[i], z_Ol[i,4]*cos(z_Ol[i,3]+bta[i])/(1-z_Ol[i,2]*c[i]))
            @NLconstraint(mdl, z_Ol[i+1,1] == z_Ol[i,1] + dt*dsdt[i]  )                                                
            @NLconstraint(mdl, z_Ol[i+1,2] == z_Ol[i,2] + dt*z_Ol[i,4]*sin(z_Ol[i,3]+bta[i])  )                       
            @NLconstraint(mdl, z_Ol[i+1,3] == z_Ol[i,3] + dt*(z_Ol[i,4]*z_Ol[i+1,5]*sin(bta[i])-dsdt[i]*c[i])  )              
            @NLconstraint(mdl, z_Ol[i+1,5] == z_Ol[i,5] + dt*u_Ol[i,3] )           
            @NLconstraint(mdl, z_Ol[i+1,4] == z_Ol[i,4] + dt*(z_Ol[i,7] - 0.5*z_Ol[i,4]))  
            @NLconstraint(mdl, z_Ol[i+1,6] == z_Ol[i,6] + dt*(z_Ol[i,4]/Lref*sin(bta[i])-c[i]*z_Ol[i,4]*cos(z_Ol[i,6]+bta[i])/(1-z_Ol[i,2]*c[i]) )  )

        end

        # Hard constraints on steering angle derivative
        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] <= 0.06)
        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] >= -0.06)
        for i=1:N-1 # Constraints on u:
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] <= 0.06)
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] >= -0.06)
        end


        # Terminal set constraints (soft)
        # ---------------------------------
        for j = 1:5
            @NLconstraint(mdl, (ParInt*(sum{coeffTermConst[i,1,j]*z_Ol[N+1,1]^(order+1-i),i=1:order}+coeffTermConst[order+1,1,j])+(1-ParInt)*(sum{coeffTermConst[i,2,j]*z_Ol[N+1,1]^(order+1-i),i=1:order}+coeffTermConst[order+1,2,j])-z_Ol[N+1,j+1])^2 <= eps[j])
        end


        # Cost functions

        # Derivative cost for both state and input derivatives
        # ---------------------------------
        @NLexpression(mdl, derivCost, sum{QderivZ[j]*(sum{(z_Ol[i,j]-z_Ol[i+1,j])^2,i=1:N}),j=1:5} +
                                          QderivU[1]*((uPrev[1,1]-u_Ol[1,1])^2+sum{(u_Ol[i,1]-u_Ol[i+1,1])^2,i=1:N-delay_a-1})+
                                          QderivU[2]*((uPrev[1,2]-u_Ol[1,2])^2+sum{(u_Ol[i,2]-u_Ol[i+1,2])^2,i=1:N-delay_df-1})+
                                          QderivU[3]*sum{(u_Ol[i+1,3]-u_Ol[i,3])^2,i=1:N-1})

        # Lane cost
        # ---------------------------------
        @NLexpression(mdl, laneCost, 100*eps[6]+1000*eps[6]^2 + 100*eps[7]+1000*eps[7]^2) 

        # Control Input cost
        # ---------------------------------
        @NLexpression(mdl, controlCost, R[1]*sum{(u_Ol[i,1])^2,i=1:N-delay_a}+
                                        R[2]*sum{(u_Ol[i,2])^2,i=1:N-delay_df})

        # Violation of soft terminal constraint (penalize soft constraint variable eps)
        # ---------------------------------         
        @NLexpression(mdl, constZTerm, sum{Q_term[j]*(eps[j]+eps[j]^2),j=1:4})

        # Terminal cost
        # ---------------------------------
        # The value of this cost determines how fast the algorithm learns. The higher this cost, the faster the control tries to reach the finish line.
        @NLexpression(mdl, costZTerm,  0.65*(ParInt*(sum{coeffTermCost[i,1]*z_Ol[N+1,1]^(order+1-i),i=1:order}+coeffTermCost[order+1,1])+
                                      (1-ParInt)*(sum{coeffTermCost[i,2]*z_Ol[N+1,1]^(order+1-i),i=1:order}+coeffTermCost[order+1,2]))) 

        # Model error cost (deviation in steering error e_ψ from reference model)
        # ---------------------------------
        @NLexpression(mdl, modelErrorCost, Q_modelError*sum{(z_Ol[j,6]-z_Ol[j,3])^2,j=1:N+1})

        # Solve model once
        @NLobjective(mdl, Min,  derivCost + constZTerm + costZTerm + laneCost + modelErrorCost  + controlCost)
        sol_stat=solve(mdl)
        println("Finished solve 1: $sol_stat")
        sol_stat=solve(mdl)
        println("Finished solve 2: $sol_stat")
        m.mdl = mdl
        m.z0 = z0
        m.coeff = coeff
        m.z_Ol = z_Ol
        m.u_Ol = u_Ol
        m.ParInt = ParInt
        m.uPrev = uPrev
        m.rhoPrev = rhoPrev

        m.coeffTermCost = coeffTermCost
        m.coeffTermConst = coeffTermConst
        m.c = c
        m.eps = eps

        m.derivCost = derivCost
        m.controlCost = controlCost
        m.modelErrorCost  = modelErrorCost
        m.costZTerm  = costZTerm
        m.constZTerm = constZTerm
        m.laneCost = laneCost

        return m
    end
end

type MpcModel_pF
    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    coeff::Array{JuMP.NonlinearParameter,1}

    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}

    derivCost::JuMP.NonlinearExpression
    costZ::JuMP.NonlinearExpression
    controlCost::JuMP.NonlinearExpression

    uPrev::Array{JuMP.NonlinearParameter,2}

    function MpcModel_pF(mpcParams::MpcParams,modelParams::ModelParams,trackCoeff::TrackCoeff)
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
        @variable( mdl, z_Ol[1:(N+1),1:6], start = 0)          # z = s, ey, epsi, v
        @variable( mdl, u_Ol[1:N,1:2], start = 0)

        # Set bounds
        z_lb_4s = ones(mpcParams.N+1,1)*[-Inf -Inf -Inf -0.5]                  # lower bounds on states
        z_ub_4s = ones(mpcParams.N+1,1)*[ Inf  Inf  Inf  1.5]                  # upper bounds
        u_lb_4s = ones(mpcParams.N,1) * [0.0  -0.3]                            # lower bounds on steering
        u_ub_4s = ones(mpcParams.N,1) * [1.2   0.3]                            # upper bounds

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

        @NLparameter(mdl, z0[i=1:6] == 0)

        @NLparameter(mdl, uPrev[1:10,1:2] == 0)

        @NLparameter(mdl, coeff[i=1:n_poly_curv+1] == 0)

        @NLexpression(mdl, c[i = 1:N], sum{coeff[j]*z_Ol[i,1]^(n_poly_curv-j+1),j=1:n_poly_curv} + coeff[n_poly_curv+1])

        # System dynamics
        setvalue(z0[4],v_ref)
        @NLconstraint(mdl, [i=1:6], z_Ol[1,i] == z0[i])         # initial condition
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
            @NLconstraint(mdl, z_Ol[i+1,6] == z_Ol[i,6] + dt*(z_Ol[i,4]/L_a*sin(bta[i])-(z_Ol[i,4]*cos(z_Ol[i,6]+bta[i])/(1-z_Ol[i,2]*c[i]))*c[i])  )               # epsi
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
        # for i=1:N+1
        #     setvalue(z_Ol[i,:],[(i-1)*dt*v_ref 0 0 v_ref 0])
        # end
        # for i=1:N
        #     setvalue(u_Ol[i,:],[0.15 0])
        # end
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
