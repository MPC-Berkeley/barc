using Polynomials

include("optimizer.jl")


type MpcModel_pF
    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    curvature::Array{JuMP.NonlinearParameter, 1}
    z_Ref::Array{JuMP.NonlinearParameter,2}

    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}

    derivCost::JuMP.NonlinearExpression
    costZ::JuMP.NonlinearExpression
    controlCost::JuMP.NonlinearExpression

    uPrev::Array{JuMP.NonlinearParameter,2}

    function MpcModel_pF(agent::Agent, track::Track)
        println("Starting creation of pf model")
        m = new()
        dt = agent.dt
        L_a = agent.l_front
        L_b = agent.l_rear

        N = size(agent.optimal_inputs, 1)
        # Q = [5.0, 0.0, 0.0, 0.1, 50.0, 0.0]  # Q (only for path following mode)
        Q = [0.0, 20.0, 1.0, 10.0]  # Q (only for path following mode)
        R = 2 * [1.0, 1.0]  # put weights on a and d_f
        # QderivZ = 10.0 * [1, 1, 1, 1, 1, 1]  # cost matrix for derivative cost of states
        QderivZ = 0.1 * [0, 1, 1, 1, 1, 1]  # cost matrix for derivative cost of states
        QderivU = 0.1 * [1, 0.1]  #NOTE Set this to [5.0, 0/40.0]              # cost matrix for derivative cost of inputs
        # delay_df = 3  # steering delay
        # delay_a = 1  # acceleration delay
        # delay_df = 1  # steering delay
        # delay_a = 1  # acceleration delay

        delay_a = agent.delay_a
        delay_df = agent.delay_df

        vPathFollowing = 1.0  # reference speed for first lap of path following
        # Q_term = 1.0 * [20.0, 1.0, 10.0, 20.0, 50.0]  # weights for terminal constraints (LMPC, for xDot,yDot,psiDot,ePsi,eY).Not used if using convex hull
        
        # Q_term_cost = 3  # scaling of Q-function
        # Q_lane = 1  # weight on the soft constraint for the lane
        # Q_vel = 1  # weight on the soft constraint for the maximum velocity
        # Q_slack = 1 * [20.0, 1.0, 10.0, 30.0, 80.0, 50.0]  #[20.0,10.0,10.0,30.0,80.0,50.0]  #vx,vy,psiDot,ePsi,eY,s
        # Q_obs = ones(Nl * selectedStates.Np)  # weight to esclude some of the old trajectories

        println("prediction h = ", N)

        v_ref       = vPathFollowing
        acc_f       = 1.0

        # n_poly_curv = trackCoeff.nPolyCurvature  # polynomial degree of curvature approximation

        # Create function-specific parameters
        z_ref::Array{Float64,2}
        z_ref = cat(2, zeros(N + 1, 3), v_ref * ones(N + 1, 1))  # Reference trajectory: path following -> stay on line and keep constant velocity
        u_Ref = zeros(N, 2)

        # Create Model
        mdl = Model(solver = IpoptSolver(print_level=0, max_cpu_time=0.09, 
                                         linear_solver="ma27"))
                                         #,linear_solver="ma57",print_user_options="yes"))

        # Create variables (these are going to be optimized)
        @variable(mdl, z_Ol[1 : (N + 1), 1 : 4], start = 0)  # z = s, ey, epsi, v
        # @variable(mdl, z_Ol[1 : (N + 1), 1 : 5], start = 0)
        @variable(mdl, u_Ol[1 : N, 1 : 2], start = 0)

        # Set bounds
        z_lb_4s = ones(N + 1, 1) * [-Inf -Inf -Inf -0.5]  # lower bounds on states
        z_ub_4s = ones(N + 1, 1) * [Inf Inf Inf 1.5]  # upper bounds
        u_lb_4s = ones(N, 1) * [0 -0.4]  # lower bounds on steering
        u_ub_4s = ones(N, 1) * [1.2 0.4]  # upper bounds

        for i = 1 : 2
            for j = 1 : N
                setlowerbound(u_Ol[j, i], u_lb_4s[j, i])
                setupperbound(u_Ol[j, i], u_ub_4s[j, i])
            end
        end

        # for i=1:4
        #     for j=1:N+1
        #         setlowerbound(z_Ol[j,i], z_lb_4s[j,i])
        #         setupperbound(z_Ol[j,i], z_ub_4s[j,i])
        #     end
        # end

        @NLparameter(mdl, z_Ref[1 : N + 1, 1 : 4] == 0)
        # @NLparameter(mdl, z0[i = 1 : 5] == 0)
        @NLparameter(mdl, z0[i = 1 : 4] == 0)
        @NLparameter(mdl, uPrev[1 : 10, 1 : 2] == 0)

        # Curvature
        # kappa = zeros(N)
        # @NLparameter(mdl, c[1 : N] == 0)
        if POLYNOMIAL_CURVATURE
            @NLparameter(mdl, coeff[i = 1 : 5 + 1] == 0)
            @NLexpression(mdl, curvature[i = 1 : N], 
                          sum{coeff[j] * z_Ol[i, 1]^(j - 1),
                          j = 2 : 6} + coeff[1])
        else
            @NLparameter(mdl, curvature[i = 1 : N] == 0)
        end

        # System dynamics
        setvalue(z0[4], v_ref)
        # @NLconstraint(mdl, [i = 1 : 5], z_Ol[1, i] == z0[i])  # initial condition
        @NLconstraint(mdl, [i = 1 : 4], z_Ol[1, i] == z0[i])  # initial condition

        for i = 1 : N
            if i <= delay_df
                @NLexpression(mdl, bta[i], atan(L_a / (L_a + L_b) * tan(uPrev[delay_df + 1 - i, 2])))
            else
                @NLexpression(mdl, bta[i], atan(L_a / (L_a + L_b) * tan(u_Ol[i - delay_df, 2])))
            end

            #=
            if i <= delay_a
                @NLconstraint(mdl, z_Ol[i + 1, 5] == z_Ol[i, 5] + dt * 
                                                     (uPrev[delay_a + 1 - i, 1] - z_Ol[i, 5]) * acc_f)  # v
            else
                @NLconstraint(mdl, z_Ol[i + 1, 5] == z_Ol[i, 5] + dt * 
                                                     (u_Ol[i - delay_a, 1] - z_Ol[i, 5]) * acc_f)  # v
            end
            =#

            # s_dot
            @NLexpression(mdl, dsdt[i], z_Ol[i, 4] * cos(z_Ol[i, 3] + bta[i]) / (1 - z_Ol[i, 2] * curvature[i]))
            # s
            @NLconstraint(mdl, z_Ol[i + 1, 1] == z_Ol[i, 1] + dt * dsdt[i])  
            @NLconstraint(mdl, z_Ol[i + 1, 2] == z_Ol[i, 2] + dt * z_Ol[i, 4] * sin(z_Ol[i, 3] + bta[i]))  # ey
            @NLconstraint(mdl, z_Ol[i + 1, 3] == z_Ol[i, 3] + dt * (z_Ol[i, 4] / L_a * sin(bta[i]) - dsdt[i] * curvature[i]))  # epsi
            # @NLconstraint(mdl, z_Ol[i + 1, 4] == z_Ol[i, 4] + dt * (z_Ol[i, 5] - 0.05 * z_Ol[i, 4]))  # v
            # v
            if i <= delay_a
                @NLconstraint(mdl, z_Ol[i + 1, 4] == z_Ol[i, 4] + dt * (uPrev[delay_a + 1 - i, 1]))
                # @NLconstraint(mdl, z_Ol[i + 1, 4] == z_Ol[i, 4] + dt * (uPrev[delay_a + 1 - i, 1] - 0.05 * z_Ol[i, 4]))
            else
                @NLconstraint(mdl, z_Ol[i + 1, 4] == z_Ol[i, 4] + dt * (u_Ol[i - delay_a, 1, 1]))
                # @NLconstraint(mdl, z_Ol[i + 1, 4] == z_Ol[i, 4] + dt * (u_Ol[i - delay_a, 1, 1] - 0.05 * z_Ol[i, 4]))
            end
        end

        @NLconstraint(mdl, u_Ol[1, 2] - uPrev[1, 2] <= 0.06)
        @NLconstraint(mdl, u_Ol[1, 2] - uPrev[1, 2] >= - 0.06)

        for i = 1 : N - 1 # Constraints on u:
            @NLconstraint(mdl, u_Ol[i + 1, 2] - u_Ol[i, 2] <= 0.06)
            @NLconstraint(mdl, u_Ol[i + 1, 2] - u_Ol[i, 2] >= - 0.06)
        end

        # Cost definitions
        # Derivative cost
        @NLexpression(mdl, derivCost, sum{QderivZ[j] * (sum{(z_Ol[i, j] - z_Ol[i + 1, j])^2, i = 1 : N}), j = 1 : 4} +
                           QderivU[1] * ((uPrev[1, 1] - u_Ol[1, 1])^2 + sum{(u_Ol[i, 1] - u_Ol[i + 1, 1])^2, i = 1 : N - delay_a - 1})+
                           QderivU[2] * ((uPrev[1, 2] - u_Ol[1, 2])^2 + sum{(u_Ol[i, 2] - u_Ol[i + 1, 2])^2, i = 1 : N - delay_df - 1}))

        # Control Input cost
        @NLexpression(mdl, controlCost, 0.5 * R[1] * sum{(u_Ol[i, 1])^2, i = 1 : N - delay_a} +
                                        0.5 * R[2] * sum{(u_Ol[i, 2])^2, i = 1 : N - delay_df})
        
        # State cost
        @NLexpression(mdl, costZ, 1.0 * sum{Q[i] * sum{(z_Ol[j, i] - z_Ref[j, i])^2,
                                  j = 2 : N + 1}, i = 1 : 4})  # Follow trajectory

        # Objective function
        @NLobjective(mdl, Min, costZ + derivCost + controlCost)

        # create first artificial solution (for warm start)
        for i = 1 : N + 1
            setvalue(z_Ol[i, :], [(i-1)*dt*v_ref 0 0 v_ref])
        end
        for i = 1 : N
            setvalue(u_Ol[i, :], [0.15 0])
        end

        # First solve
        sol_stat = solve(mdl)
        println("Finished solve 1: $sol_stat")
        sol_stat = solve(mdl)
        println("Finished solve 2: $sol_stat")
        
        m.mdl = mdl
        m.z0 = z0
        m.z_Ref = z_Ref
        if POLYNOMIAL_CURVATURE
            m.curvature = coeff
        else
            m.curvature = curvature
        end
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
    curvature::Array{JuMP.NonlinearParameter, 1}

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

    # function MpcModel_convhull(mpcParams::MpcParams,mpcCoeff::MpcCoeff,modelParams::ModelParams,trackCoeff::TrackCoeff,selectedStates::SelectedStates)
    function MpcModel_convhull(agent::Agent, track::Track)

        m = new()

        #### Initialize parameters
        dt = agent.dt
        L_a = agent.l_front
        L_b = agent.l_rear

        N = size(agent.optimal_inputs, 1)

        ey_max      = track.width / 2           # bound for the state ey (distance from the center track). It is set as half of the width of the track for obvious reasons
        # n_poly_curv = trackCoeff.nPolyCurvature    # polynomial degree for curvature approximation
        # v_max       = 1.7                            # maximum allowed velocity

        delay_a = agent.delay_a
        delay_df = agent.delay_df

        #=
        # tuning for fast laps
        Q = [0.0, 10.0, 1.0, 10.0]  # Q (only for path following mode)
        R = 0.0 * [1.0, 1.0]  # put weights on a and d_f
        QderivZ = 1.0 * [1, 1, 1, 1, 1, 1.0]  # cost matrix for derivative cost of states
        QderivU = 1.0 * 1.0 * 1.0 * [15.0, 0.1]        

        vPathFollowing = 1.0  # reference speed for first lap of path following
        
        Q_term_cost = 1.0  # scaling of Q-function
        Q_lane = 16.0
        Q_slack = 1.0 * [100.0, 1.0, 1.0, 1.0, 5.0, 10.0]
        =#

        Q = [0.0, 10.0, 1.0, 10.0]  # Q (only for path following mode)
        R = 0.0 * [1.0, 1.0]  # put weights on a and d_f
        QderivZ = 1.0 * [1, 1, 1, 1, 1, 1.0]  # cost matrix for derivative cost of states
        QderivU = 1.0 * 1.0 * 1.0 * [10.0, 0.1]

        vPathFollowing = 1.0  # reference speed for first lap of path following
        
        Q_term_cost = 1.0  # scaling of Q-function
        Q_lane = 16.0
        # Q_slack = 1.1 * 1.0 * [100.0, 1.0, 1.0, 1.0, 4.0 * 5.0, 10.0]
        Q_slack = 1.0 * 1.0 * [100.0, 1.0, 1.0, 1.0, 0.5 * 0.5 * 4.0 * 5.0, 10.0]
        # Q_slack = 1.1 * 1.0 * [100.0, 1.0, 1.0, 0.1 * 1.0, 0.1 * 4.0 * 5.0, 10.0]

        println("prediction horizon = ", N)

        num_considered_states = size(agent.selected_states_s)[1]
        acc_f = 1.0  # if this is on, the agent accelerates and decelerates the whole time
        # acc_f = 0.0
        # acc_f = 100  # if this is on, the agent accelerates and decelerates the whole time
        # acc_f = 0.1  # if this is on, the agent accelerates and decelerates the whole time


        mdl = Model(solver = IpoptSolver(print_level=0, max_cpu_time=0.09, 
                                         linear_solver="ma27"))#,check_derivatives_for_naninf="yes"))#,linear_solver="ma57",print_user_options="yes"))

        # @variable( mdl, z_Ol[1 : (N + 1), 1 : 7])
        @variable( mdl, z_Ol[1 : (N + 1), 1 : 6])
        @variable( mdl, u_Ol[1 : N, 1 : 2])
        @variable( mdl, eps_lane[1 : N + 1] >= 0)  # eps for soft lane constraints
        @variable( mdl, alpha[1 : num_considered_states] >= 0)  # coefficients of the convex hull
        # @variable( mdl, eps_alpha[1:6] >=0)  # eps for soft constraint on alpha
        # @variable( mdl, eps_vel[1:N+1]>=0)  # eps for soft constraint on velocity

        z_lb_6s = ones(N + 1, 1) * [0.1 -Inf -Inf -Inf -Inf -Inf -Inf]  # lower bounds on states
        z_ub_6s = ones(N + 1, 1) * [3.5  Inf Inf  Inf  Inf  Inf Inf]  # upper bounds
        u_lb_6s = ones(N, 1) * [-1.3  -0.4]  # lower bounds on steering
        u_ub_6s = ones(N, 1) * [3.0   0.4]  # upper bounds

        for i = 1 : 2
            for j = 1 : N
                setlowerbound(u_Ol[j, i], u_lb_6s[j, i])
                setupperbound(u_Ol[j, i], u_ub_6s[j, i])
            end
        end

        # for i = 1 : 7
        for i = 1 : 6
            for j = 1 : N + 1
                setlowerbound(z_Ol[j, i], z_lb_6s[j, i])
                setupperbound(z_Ol[j, i], z_ub_6s[j, i])
            end
        end

        # @NLparameter(mdl, z0[i = 1 : 7] == 0)
        @NLparameter(mdl, z0[i = 1 : 6] == 0)
        @NLparameter(mdl, c_Vx[i = 1 : 3]  == 0)
        @NLparameter(mdl, c_Vy[i = 1 : 4]  == 0)
        @NLparameter(mdl, c_Psi[i = 1 : 3] == 0)
        @NLparameter(mdl, uPrev[1 : 10, 1 : 2] == 0)
        @NLparameter(mdl, selStates[1 : num_considered_states, 1 : 6] == 0)  # states from the previous trajectories selected in "convhullStates"
        @NLparameter(mdl, statesCost[1 : num_considered_states] == 0)  # costs of the states selected in "convhullStates"
        
        if POLYNOMIAL_CURVATURE
            @NLparameter(mdl, coeff[i = 1 : 5 + 1] == 0)
            @NLexpression(mdl, curvature[i = 1 : N], 
                          sum{coeff[j] * z_Ol[i, 1]^(j - 1),
                          j = 2 : 6} + coeff[1])
        else
            @NLparameter(mdl, curvature[i = 1 : N] == 0)
        end

        # Conditions for first solve:
        setvalue(z0[1], 1)
        setvalue(c_Vx[3], 0.1)

        # @NLconstraint(mdl, [i = 1 : 7], z_Ol[1, i] == z0[i])
        @NLconstraint(mdl, [i = 1 : 6], z_Ol[1, i] == z0[i])

        @NLconstraint(mdl, [i = 2 : N + 1], z_Ol[i, 5] <= ey_max + eps_lane[i])
        @NLconstraint(mdl, [i = 2 : N + 1], z_Ol[i, 5] >= - ey_max - eps_lane[i])
        # @NLconstraint(mdl,[i = 1:(N+1)], z_Ol[i,4] <= v_max + eps_vel[i] )  # soft constraint on maximum velocity
        @NLconstraint(mdl, sum{alpha[i], i = 1 : num_considered_states} == 1)  # constraint on the coefficients of the convex hull
        
        #=
        for i = 1 : 6                                                                                                       
            @NLconstraint(mdl, z_Ol[N + 1, i] == sum{alpha[j] * selStates[j, i], 
                                                 j = 1 : num_considered_states})         
        end 
        =#
        
        #for n = 1:6
            #@NLconstraint(mdl,z_Ol[N+1,n] == sum{alpha[j]*selStates[j,n],j=1:num_considered_states})  # terminal constraint
            #@NLconstraint(mdl,z_Ol[N+1,n] >= sum{alpha[j]*selStates[j,n],j=1:num_considered_states}-eps_alpha[n])  
            #@NLconstraint(mdl,z_Ol[N+1,n] <= sum{alpha[j]*selStates[j,n],j=1:num_considered_states}+eps_alpha[n])
        #end  

        @NLexpression(mdl, dsdt[i = 1 : N], (z_Ol[i, 1] * cos(z_Ol[i, 4]) - 
                                             z_Ol[i, 2] * sin(z_Ol[i, 4])) /
                                             (1 - z_Ol[i, 5] * curvature[i]))
        
        println("Initializing model...")

        # System dynamics
        for i = 1 : N
            if i <= delay_df
                # yDot
                @NLconstraint(mdl, z_Ol[i + 1, 2] == z_Ol[i, 2] + 
                                                     c_Vy[1] * z_Ol[i, 2] / z_Ol[i, 1] + 
                                                     c_Vy[2] * z_Ol[i, 1] * z_Ol[i, 3] + 
                                                     c_Vy[3] * z_Ol[i, 3] / z_Ol[i, 1] + 
                                                     c_Vy[4] * uPrev[delay_df + 1 - i, 2])  
                # psiDot
                @NLconstraint(mdl, z_Ol[i + 1, 3] == z_Ol[i, 3] + 
                                                     c_Psi[1] * z_Ol[i, 3] / z_Ol[i, 1] +
                                                     c_Psi[2] * z_Ol[i, 2] / z_Ol[i, 1] + 
                                                     c_Psi[3] * uPrev[delay_df + 1 - i, 2])
            else
                # yDot
                @NLconstraint(mdl, z_Ol[i + 1, 2] == z_Ol[i, 2] + 
                                                     c_Vy[1] * z_Ol[i, 2] / z_Ol[i, 1] + 
                                                     c_Vy[2] * z_Ol[i, 1] * z_Ol[i, 3] + 
                                                     c_Vy[3] * z_Ol[i, 3] / z_Ol[i, 1] + 
                                                     c_Vy[4] * u_Ol[i - delay_df, 2]) 
                # psiDot
                @NLconstraint(mdl, z_Ol[i + 1, 3] == z_Ol[i, 3] + 
                                                     c_Psi[1] * z_Ol[i, 3] / z_Ol[i, 1] + 
                                                     c_Psi[2] * z_Ol[i, 2] / z_Ol[i, 1] + 
                                                     c_Psi[3] * u_Ol[i - delay_df, 2])                    
            end
            # xDot 
            if i <= delay_a
                @NLconstraint(mdl, z_Ol[i + 1, 1] == z_Ol[i, 1] + 
                                                 c_Vx[1] * z_Ol[i, 2] * z_Ol[i, 3] + 
                                                 c_Vx[2] * z_Ol[i, 1] + 
                                                 c_Vx[3] * uPrev[delay_a + 1 - i, 1]) 
            else
                @NLconstraint(mdl, z_Ol[i + 1, 1] == z_Ol[i, 1] + 
                                                 c_Vx[1] * z_Ol[i, 2] * z_Ol[i, 3] + 
                                                 c_Vx[2] * z_Ol[i, 1] + 
                                                 c_Vx[3] * u_Ol[i - delay_a, 1]) 
            end

            # ePsi
            @NLconstraint(mdl, z_Ol[i + 1, 4] == z_Ol[i, 4] + dt * 
                                                 (z_Ol[i, 3] - dsdt[i] * curvature[i])) 
            # eY                                                                                
            @NLconstraint(mdl, z_Ol[i + 1, 5] == z_Ol[i, 5] + dt * 
                                                 (z_Ol[i, 1] * sin(z_Ol[i, 4]) + 
                                                  z_Ol[i, 2] * cos(z_Ol[i, 4])))           
            # s                                           
            @NLconstraint(mdl, z_Ol[i + 1, 6] == z_Ol[i, 6] + dt * dsdt[i])                                                                                                
        end

        @NLconstraint(mdl, u_Ol[1, 2] - uPrev[1, 2] <= 0.12)
        @NLconstraint(mdl, u_Ol[1, 2] - uPrev[1, 2] >= - 0.12)

        for i = 1 : N - 1 # Constraints on u:
            @NLconstraint(mdl, u_Ol[i + 1, 2] - u_Ol[i, 2] <= 0.12)
            @NLconstraint(mdl, u_Ol[i + 1, 2] - u_Ol[i, 2] >= - 0.12)
        end

        # Cost functions
        # Derivative cost
        @NLexpression(mdl, derivCost, sum{QderivZ[j] * (sum{(z_Ol[i, j] - z_Ol[i + 1, j])^2, i = 1 : N}), j = 1 : 6} +
                                          QderivU[1] * ((uPrev[1, 1] - u_Ol[1, 1])^2 + 
                                          sum{(u_Ol[i, 1] - u_Ol[i + 1, 1])^2, i = 1 : N - 1}) +
                                          QderivU[2] * ((uPrev[1, 2] - u_Ol[1, 2])^2 + 
                                          sum{(u_Ol[i, 2] - u_Ol[i + 1, 2])^2, i = 1 : N - delay_df - 1}))        

        # Control Input cost
        @NLexpression(mdl, controlCost, R[1] * sum{(u_Ol[i, 1])^2, i = 1 : N - delay_a} +
                                        R[2] * sum{(u_Ol[i, 2])^2, i = 1 : N - delay_df})

        # Lane cost (soft)
        @NLexpression(mdl, laneCost, Q_lane * sum{1.0 * eps_lane[i] + 
                                                  20.0 * eps_lane[i]^2, 
                                                  i = 2 : N + 1})
        # @NLexpression(mdl, laneCost, Q_lane * sum{10.0 * eps_lane[i] + 
        #                                         100.0 * eps_lane[i]^2, 
        #                                          i = 2 : N + 1})

        # Terminal Cost
        @NLexpression(mdl, terminalCost , sum{alpha[i] * statesCost[i], i = 1 : num_considered_states})

        # Slack cost (soft)
        #@NLexpression(mdl, slackCost, sum{50*eps_alpha[i]+500*eps_alpha[i]^2,i=1:6})

        # Slack cost on vx
        @NLexpression(mdl, slackVx, (z_Ol[N + 1, 1] - sum{alpha[j] * selStates[j, 1], j = 1 : num_considered_states})^2)

        # Slack cost on vy
        @NLexpression(mdl, slackVy, (z_Ol[N + 1, 2] - sum{alpha[j] * selStates[j, 2], j = 1 : num_considered_states})^2)

        # Slack cost on Psi dot
        @NLexpression(mdl, slackPsidot, (z_Ol[N + 1, 3] - sum{alpha[j] * selStates[j, 3], j = 1 : num_considered_states})^2)

        # Slack cost on ePsi
        @NLexpression(mdl, slackEpsi, (z_Ol[N + 1, 4] - sum{alpha[j] * selStates[j, 4], j = 1 : num_considered_states})^2)

        # Slack cost on ey
        @NLexpression(mdl, slackEy, (z_Ol[N + 1, 5] - sum{alpha[j] * selStates[j, 5], j = 1 : num_considered_states})^2)

        # Slack cost on s
        @NLexpression(mdl, slackS, (z_Ol[N + 1, 6] - sum{alpha[j] * selStates[j, 6], j = 1 : num_considered_states})^2)

        # Velocity Cost
        #@NLexpression(mdl, velocityCost , Q_vel*sum{10.0*eps_vel[i]+100.0*eps_vel[i]^2 ,i=2:N+1})

        # Overall Cost function (objective of the minimization)
        #@NLobjective(mdl, Min, derivCost + laneCost + controlCost + terminalCost )#+ slackCost)#+ velocityCost)
        @NLobjective(mdl, Min, derivCost + laneCost + Q_term_cost * terminalCost + controlCost + 
                               Q_slack[1] * slackVx + Q_slack[2] * slackVy + 
                               Q_slack[3] * slackPsidot + Q_slack[4] * slackEpsi + 
                               Q_slack[5] * slackEy + Q_slack[6] * slackS) #+ controlCost

        sol_stat = solve(mdl)
        println("Finished solve 1 convhull mpc: $sol_stat")
        sol_stat = solve(mdl)
        println("Finished solve 2 convhull mpc: $sol_stat")

        m.mdl = mdl
        m.z0 = z0
        if POLYNOMIAL_CURVATURE
            m.curvature = coeff
        else
            m.curvature = curvature
        end
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
    curvature::Array{JuMP.NonlinearParameter, 1}
    weights_obstacles::Array{JuMP.NonlinearParameter, 1}
    weights_obstacle::JuMP.NonlinearParameter
    safety_weight::JuMP.NonlinearParameter
    adv_states_s::Array{JuMP.NonlinearParameter, 2}
    current_lap::JuMP.NonlinearParameter
    adv_lap::JuMP.NonlinearParameter

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
    obstacle_cost::JuMP.NonlinearExpression
    progress_cost::JuMP.NonlinearExpression
    slackVx::JuMP.NonlinearExpression
    slackVy::JuMP.NonlinearExpression
    slackPsidot::JuMP.NonlinearExpression
    slackEpsi::JuMP.NonlinearExpression
    slackEy::JuMP.NonlinearExpression
    slackS::JuMP.NonlinearExpression
    #slackCost::JuMP.NonlinearExpression
    #velocityCost::JuMP.NonlinearExpression

    # function MpcModel_convhull(mpcParams::MpcParams,mpcCoeff::MpcCoeff,modelParams::ModelParams,trackCoeff::TrackCoeff,selectedStates::SelectedStates)
    function MpcModel_obstacle(agent::Agent, track::Track)

        m = new()

        #### Initialize parameters
        dt = agent.dt
        L_a = agent.l_front
        L_b = agent.l_rear
        width = agent.width

        N = size(agent.optimal_inputs, 1)

        ey_max      = track.width / 2           # bound for the state ey (distance from the center track). It is set as half of the width of the track for obvious reasons
        # n_poly_curv = trackCoeff.nPolyCurvature    # polynomial degree for curvature approximation
        if agent.index == 2
            v_max       = 1.5                            # maximum allowed velocity
        else 
            v_max = 100.0
        end

        delay_a = agent.delay_a
        delay_df = agent.delay_df

        Q = [0.0, 10.0, 1.0, 10.0]  # Q (only for path following mode)
        R = 0.0 * [1.0, 1.0]  # put weights on a and d_f
        QderivZ = 1.0 * [1, 1, 1, 1, 1, 1.0]  # cost matrix for derivative cost of states
        QderivU = 1.0 * 1.0 * 1.0 * [2 * 15.0, 0.1]

        vPathFollowing = 1.0  # reference speed for first lap of path following
        
        Q_term_cost = 1.0  # scaling of Q-function
        Q_lane = 0.5 * 16.0
        Q_slack = 1.1 * 1.0 * [100.0, 1.0, 1.0, 1.0, 1.0 * 5.0, 10.0]
        Q_vel = 1.0

        weights_progress = 0.5 * (- 1.0 * ones(N + 1))

        println("prediction horizon = ", N)

        num_considered_states = size(agent.selected_states_s)[1]
        acc_f = 1.0  # if this is on, the agent accelerates and decelerates the whole time
        # acc_f = 0.0
        # acc_f = 100  # if this is on, the agent accelerates and decelerates the whole time
        # acc_f = 0.1  # if this is on, the agent accelerates and decelerates the whole time


        mdl = Model(solver = IpoptSolver(print_level=0, max_cpu_time=0.09, 
                                         linear_solver="ma27"))#,check_derivatives_for_naninf="yes"))#,linear_solver="ma57",print_user_options="yes"))

        # @variable( mdl, z_Ol[1 : (N + 1), 1 : 7])
        @variable( mdl, z_Ol[1 : (N + 1), 1 : 6])
        @variable( mdl, u_Ol[1 : N, 1 : 2])
        @variable( mdl, eps_lane[1 : N + 1] >= 0)  # eps for soft lane constraints
        @variable( mdl, alpha[1 : num_considered_states] >= 0)  # coefficients of the convex hull
        # @variable( mdl, eps_alpha[1:6] >=0)  # eps for soft constraint on alpha
        @variable( mdl, eps_vel[1:N+1]>=0)  # eps for soft constraint on velocity

        z_lb_6s = ones(N + 1, 1) * [0.1 -Inf -Inf -Inf -Inf -Inf -Inf]  # lower bounds on states
        z_ub_6s = ones(N + 1, 1) * [3.5  Inf Inf  Inf  Inf  Inf Inf]  # upper bounds
        u_lb_6s = ones(N, 1) * [-1.3  -0.3]  # lower bounds on steering
        u_ub_6s = ones(N, 1) * [2.0   0.3]  # upper bounds

        for i = 1 : 2
            for j = 1 : N
                setlowerbound(u_Ol[j, i], u_lb_6s[j, i])
                setupperbound(u_Ol[j, i], u_ub_6s[j, i])
            end
        end

        # for i = 1 : 7
        for i = 1 : 6
            for j = 1 : N + 1
                setlowerbound(z_Ol[j, i], z_lb_6s[j, i])
                setupperbound(z_Ol[j, i], z_ub_6s[j, i])
            end
        end

        # @NLparameter(mdl, z0[i = 1 : 7] == 0)
        @NLparameter(mdl, z0[i = 1 : 6] == 0)
        @NLparameter(mdl, c_Vx[i = 1 : 3]  == 0)
        @NLparameter(mdl, c_Vy[i = 1 : 4]  == 0)
        @NLparameter(mdl, c_Psi[i = 1 : 3] == 0)
        @NLparameter(mdl, uPrev[1 : 10, 1 : 2] == 0)
        @NLparameter(mdl, selStates[1 : num_considered_states, 1 : 6] == 0)  # states from the previous trajectories selected in "convhullStates"
        @NLparameter(mdl, statesCost[1 : num_considered_states] == 0)  # costs of the states selected in "convhullStates"
        @NLparameter(mdl, progress[1 : N + 1] == 0)
        @NLparameter(mdl, weights_obstacles[1 : num_considered_states] == 1)
        @NLparameter(mdl, weights_obstacle == 0)
        @NLparameter(mdl, safety_weight == 1)

        @NLparameter(mdl, adv_states_s[1 : (N + 1), 1 : 6] == 0)
        @NLparameter(mdl, current_lap == 0)
        @NLparameter(mdl, adv_lap == 0)


        if POLYNOMIAL_CURVATURE
            @NLparameter(mdl, coeff[i = 1 : 5 + 1] == 0)
            @NLexpression(mdl, curvature[i = 1 : N], 
                          sum{coeff[j] * z_Ol[i, 1]^(j - 1),
                          j = 2 : 6} + coeff[1])
        else
            @NLparameter(mdl, curvature[i = 1 : N] == 0)
        end

        # Conditions for first solve:
        setvalue(z0[1], 1)
        setvalue(c_Vx[3], 0.1)

        # @NLconstraint(mdl, [i = 1 : 7], z_Ol[1, i] == z0[i])
        @NLconstraint(mdl, [i = 1 : 6], z_Ol[1, i] == z0[i])

        @NLconstraint(mdl, [i = 2 : N + 1], z_Ol[i, 5] <= ey_max + eps_lane[i])
        @NLconstraint(mdl, [i = 2 : N + 1], z_Ol[i, 5] >= - ey_max - eps_lane[i])
        @NLconstraint(mdl,[i = 1:(N+1)], z_Ol[i,1] <= v_max + eps_vel[i] )  # soft constraint on maximum velocity
        @NLconstraint(mdl, sum{alpha[i], i = 1 : num_considered_states} == 1)  # constraint on the coefficients of the convex hull

        
        @NLexpression(mdl, dsdt[i = 1 : N], (z_Ol[i, 1] * cos(z_Ol[i, 4]) - 
                                             z_Ol[i, 2] * sin(z_Ol[i, 4])) /
                                             (1 - z_Ol[i, 5] * curvature[i]))
        
        println("Initializing model...")

        # System dynamics
        for i = 1 : N
            if i <= delay_df
                # yDot
                @NLconstraint(mdl, z_Ol[i + 1, 2] == z_Ol[i, 2] + 
                                                     c_Vy[1] * z_Ol[i, 2] / z_Ol[i, 1] + 
                                                     c_Vy[2] * z_Ol[i, 1] * z_Ol[i, 3] + 
                                                     c_Vy[3] * z_Ol[i, 3] / z_Ol[i, 1] + 
                                                     c_Vy[4] * uPrev[delay_df + 1 - i, 2])  
                # psiDot
                @NLconstraint(mdl, z_Ol[i + 1, 3] == z_Ol[i, 3] + 
                                                     c_Psi[1] * z_Ol[i, 3] / z_Ol[i, 1] +
                                                     c_Psi[2] * z_Ol[i, 2] / z_Ol[i, 1] + 
                                                     c_Psi[3] * uPrev[delay_df + 1 - i, 2])
            else
                # yDot
                @NLconstraint(mdl, z_Ol[i + 1, 2] == z_Ol[i, 2] + 
                                                     c_Vy[1] * z_Ol[i, 2] / z_Ol[i, 1] + 
                                                     c_Vy[2] * z_Ol[i, 1] * z_Ol[i, 3] + 
                                                     c_Vy[3] * z_Ol[i, 3] / z_Ol[i, 1] + 
                                                     c_Vy[4] * u_Ol[i - delay_df, 2]) 
                # psiDot
                @NLconstraint(mdl, z_Ol[i + 1, 3] == z_Ol[i, 3] + 
                                                     c_Psi[1] * z_Ol[i, 3] / z_Ol[i, 1] + 
                                                     c_Psi[2] * z_Ol[i, 2] / z_Ol[i, 1] + 
                                                     c_Psi[3] * u_Ol[i - delay_df, 2])                    
            end
            # xDot 
            if i <= delay_a
                @NLconstraint(mdl, z_Ol[i + 1, 1] == z_Ol[i, 1] + 
                                                 c_Vx[1] * z_Ol[i, 2] * z_Ol[i, 3] + 
                                                 c_Vx[2] * z_Ol[i, 1] + 
                                                 c_Vx[3] * uPrev[delay_a + 1 - i, 1]) 
            else
                @NLconstraint(mdl, z_Ol[i + 1, 1] == z_Ol[i, 1] + 
                                                 c_Vx[1] * z_Ol[i, 2] * z_Ol[i, 3] + 
                                                 c_Vx[2] * z_Ol[i, 1] + 
                                                 c_Vx[3] * u_Ol[i - delay_a, 1]) 
            end

            # ePsi
            @NLconstraint(mdl, z_Ol[i + 1, 4] == z_Ol[i, 4] + dt * 
                                                 (z_Ol[i, 3] - dsdt[i] * curvature[i])) 
            # eY                                                                                
            @NLconstraint(mdl, z_Ol[i + 1, 5] == z_Ol[i, 5] + dt * 
                                                 (z_Ol[i, 1] * sin(z_Ol[i, 4]) + 
                                                  z_Ol[i, 2] * cos(z_Ol[i, 4])))           
            # s                                           
            @NLconstraint(mdl, z_Ol[i + 1, 6] == z_Ol[i, 6] + dt * dsdt[i])                                                                                                
        end

        @NLconstraint(mdl, u_Ol[1, 2] - uPrev[1, 2] <= 0.12)
        @NLconstraint(mdl, u_Ol[1, 2] - uPrev[1, 2] >= - 0.12)

        for i = 1 : N - 1 # Constraints on u:
            @NLconstraint(mdl, u_Ol[i + 1, 2] - u_Ol[i, 2] <= 0.12)
            @NLconstraint(mdl, u_Ol[i + 1, 2] - u_Ol[i, 2] >= - 0.12)
        end

        # Cost functions
        # Derivative cost
        @NLexpression(mdl, derivCost, sum{QderivZ[j] * (sum{(z_Ol[i, j] - z_Ol[i + 1, j])^2, i = 1 : N}), j = 1 : 6} +
                                          QderivU[1] * ((uPrev[1, 1] - u_Ol[1, 1])^2 + 
                                          sum{(u_Ol[i, 1] - u_Ol[i + 1, 1])^2, i = 1 : N - delay_a - 1}) +
                                          QderivU[2] * ((uPrev[1, 2] - u_Ol[1, 2])^2 + 
                                          sum{(u_Ol[i, 2] - u_Ol[i + 1, 2])^2, i = 1 : N - delay_df - 1}))        

        # Control Input cost
        @NLexpression(mdl, controlCost, R[1] * sum{(u_Ol[i, 1])^2, i = 1 : N - delay_a} +
                                        R[2] * sum{(u_Ol[i, 2])^2, i = 1 : N - delay_df})

        # Lane cost (soft)
        @NLexpression(mdl, laneCost, Q_lane * sum{1.0 * eps_lane[i] + 
                                                  20.0 * eps_lane[i]^2, 
                                                  i = 2 : N + 1})
        # @NLexpression(mdl, laneCost, Q_lane * sum{10.0 * eps_lane[i] + 
        #                                          100.0 * eps_lane[i]^2, 
        #                                          i = 2 : N + 1})

        # Terminal Cost
        @NLexpression(mdl, terminalCost , sum{weights_obstacles[i] * alpha[i] * 
                                              statesCost[i], i = 1 : num_considered_states})

        # Slack cost (soft)
        #@NLexpression(mdl, slackCost, sum{50*eps_alpha[i]+500*eps_alpha[i]^2,i=1:6})

        # Slack cost on vx
        @NLexpression(mdl, slackVx, (z_Ol[N + 1, 1] - sum{alpha[j] * selStates[j, 1], j = 1 : num_considered_states})^2)

        # Slack cost on vy
        @NLexpression(mdl, slackVy, (z_Ol[N + 1, 2] - sum{alpha[j] * selStates[j, 2], j = 1 : num_considered_states})^2)

        # Slack cost on Psi dot
        @NLexpression(mdl, slackPsidot, (z_Ol[N + 1, 3] - sum{alpha[j] * selStates[j, 3], j = 1 : num_considered_states})^2)

        # Slack cost on ePsi
        @NLexpression(mdl, slackEpsi, (z_Ol[N + 1, 4] - sum{alpha[j] * selStates[j, 4], j = 1 : num_considered_states})^2)

        # Slack cost on ey
        @NLexpression(mdl, slackEy, (z_Ol[N + 1, 5] - sum{alpha[j] * selStates[j, 5], j = 1 : num_considered_states})^2)

        # Slack cost on s
        @NLexpression(mdl, slackS, (z_Ol[N + 1, 6] - sum{alpha[j] * selStates[j, 6], j = 1 : num_considered_states})^2)

        # Velocity Cost
        @NLexpression(mdl, velocityCost , Q_vel*sum{10.0*eps_vel[i]+100.0*eps_vel[i]^2 ,i=2:N+1})

        radius_s = zeros(NUM_AGENTS - 1)
        radius_e_y = zeros(NUM_AGENTS - 1)

        for i = 1 : NUM_AGENTS - 1
            radius_s[i] = 2 * L_a
            radius_e_y[i] = 2 * width / 2
        end

        @NLexpression(mdl, obstacle_cost, weights_obstacle * 
                      sum{- log(safety_weight * (((z_Ol[j, 6] - adv_states_s[j, 1]) / radius_s[1])^2 + 
                                ((z_Ol[j, 5] - adv_states_s[j, 2]) / radius_e_y[1])^2 - 1)), 
                          j = 1 : (N + 1)})

        @NLexpression(mdl, progress_cost, sum{weights_progress[i] * 
                      ((z_Ol[i, 6] + (current_lap - 1) * track.total_length) - 
                       (adv_states_s[i, 1] + (adv_lap - 1) * track.total_length)), i = 2 : N + 1})

        # Overall Cost function (objective of the minimization)
        #@NLobjective(mdl, Min, derivCost + laneCost + controlCost + terminalCost )#+ slackCost)#+ velocityCost)
        @NLobjective(mdl, Min, derivCost + laneCost + Q_term_cost * terminalCost + controlCost + velocityCost +
                               Q_slack[1] * slackVx + Q_slack[2] * slackVy + 
                               Q_slack[3] * slackPsidot + Q_slack[4] * slackEpsi + 
                               Q_slack[5] * slackEy + Q_slack[6] * slackS +
                               obstacle_cost + progress_cost) #+ controlCost

        sol_stat = solve(mdl)
        println("Finished solve 1 convhull mpc: $sol_stat")
        sol_stat = solve(mdl)
        println("Finished solve 2 convhull mpc: $sol_stat")

        m.mdl = mdl
        m.z0 = z0
        if POLYNOMIAL_CURVATURE
            m.curvature = coeff
        else
            m.curvature = curvature
        end
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
        m.obstacle_cost = obstacle_cost

        m.adv_states_s = adv_states_s
        m.current_lap = current_lap
        m.adv_lap = adv_lap
        m.obstacle_cost = obstacle_cost
        m.progress_cost = progress_cost
        m.weights_obstacle = weights_obstacle
        m.weights_obstacles = weights_obstacles
        m.safety_weight = safety_weight

        m.slackVx     = slackVx
        m.slackVy     = slackVy
        m.slackPsidot = slackPsidot
        m.slackEpsi   = slackEpsi
        m.slackEy     = slackEy
        m.slackS      = slackS

        return m
    end
end


# function solveMpcProblem_pathFollow(mdl::MpcModel_pF,mpcSol::MpcSol,mpcParams_pF::MpcParams,trackCoeff::TrackCoeff,posInfo::PosInfo,
#                                    modelParams::ModelParams,zCurr::Array{Float64},uPrev::Array{Float64},lapStatus::LapStatus)
function solveMpcProblem_pathFollow(mdl::MpcModel_pF, optimizer::Optimizer, agent::Agent, 
                                    track::Track, reference::Array{Float64})
    # Load Parameters
    # v_ref = 1.0
    # ey_ref = 0.0 # 0.35
    N = size(agent.optimal_inputs, 1) 
    kappa = zeros(N)
    dt = agent.dt
    num_considered_states = size(agent.selected_states_s)[1]

    iteration = agent.current_iteration
    current_s = agent.states_s[iteration, :]
    s = current_s[1]
    e_y = current_s[2]
    e_psi = current_s[3]
    v = norm(current_s[5 : 6])

    try
        agent.acc = (getvalue(mdl.z_Ol))[5, 2]
    catch
        println("solution states have wrong size")
        agent.acc = 0.0
    end
    
    acc0 = agent.acc 
    # zCurr = [s, e_y, e_psi, v, acc0]
    zCurr = [s, e_y, e_psi, v]

    u_length = 10
    u_prev = zeros(u_length, 2)
    
    if iteration <= u_length
        u_prev[1 : iteration - 1, :] = agent.inputs[iteration - 1 : - 1 : 1, :] 
        current_lap = agent.current_lap
        if current_lap > 1
            previous_lap = current_lap - 1 + NUM_LOADED_LAPS
            needed_iter = agent.iterations_needed[previous_lap]
            indeces = needed_iter : - 1 : needed_iter - (u_length - iteration)
            u_prev[iteration : end, :] = squeeze(agent.previous_inputs[previous_lap, indeces, :], 1)
        end
    else
        u_prev = agent.inputs[iteration - 1 : - 1 : iteration - u_length, :]
    end         
    #=
    lap = agent.current_lap
    indeces = num_considered_states + iteration - 1 : - 1 : iteration - u_length + num_considered_states
    u_prev = squeeze(agent.previous_inputs[lap, indeces, :], 1)
    =#

    uPrev = u_prev
    # println(uPrev)

    if POLYNOMIAL_CURVATURE
        order = 5
        n_prev = 20
        n_after = 40
        total_num = n_prev + 1 + n_after
        num_coords = size(track.s_coord, 1)
        # s = 4
        # s = 5.746450616402124
        track_index = findmin(abs(track.s_coord - s))[2]
        println("Closest s: ", track.s_coord[track_index])
        s_data = zeros(total_num)
        kappa_data = zeros(total_num)

        if track_index - n_prev < 1
            difference = abs(track_index - n_prev)
            s_part_1 = track.s_coord[end - difference : end] - track.total_length
            s_part_2 = track.s_coord[1 : total_num - difference - 1]
            s_data = [s_part_1; s_part_2]
            kappa_part_1 = track.curvature[end - difference : end]
            kappa_part_2 = track.curvature[1 : total_num - difference - 1]
            kappa_data = [kappa_part_1; kappa_part_2]
        elseif track_index + n_after > num_coords
            difference = track_index + n_after - num_coords
            s_part_1 = track.s_coord[track_index - n_prev : end]
            s_part_2 = track.s_coord[1 : difference] + track.total_length
            s_data = [s_part_1; s_part_2]
            kappa_part_1 = track.curvature[track_index - n_prev : end]
            kappa_part_2 = track.curvature[1 : difference]
            kappa_data = [kappa_part_1; kappa_part_2]
        else 
            s_data = track.s_coord[track_index - n_prev : track_index + n_after]
            kappa_data = track.curvature[track_index - n_prev : track_index + n_after]
        end

        curvature_approx = polyfit(s_data, kappa_data, order)
        indeces = collect(track_index - n_prev : track_index + n_after)
        kappa = coeffs(curvature_approx)
        # println("coefficents: ", kappa)
        # println(curvature_approx)

        #=
        app_curvature = zeros(indeces, Float64)
        counter = 1
        for i = track_index - n_prev : track_index + n_after
            evaluated_poly = polyval(curvature_approx, i * 0.1)
            # println("app curvautre: ", typeof(app_curvature[counter]))
            # println(typeof(evaluated_poly))
            app_curvature[counter] = evaluated_poly
            # println("Error: ", abs(evaluated_poly - track.curvature[i]))
            counter += 1
        end
        
        println("CURVATURE APPRXOIMATION: ", kappa)
        figure("test")
        println(size(indeces * 0.1))
        println(size(kappa_data))
        plot(indeces * 0.1, kappa_data, "ro")
        plot(indeces * 0.1, app_curvature, "bo")
        plt[:show]()
        =#
    else
        for i = 1 : N
            # println(agent.predicted_s[i + 1, 1])
            s_curve = max(0.0, agent.predicted_s[i + 1, 1])
            kappa[i] = get_curvature(track, s_curve)
        end

        #=
        for i = 1 : N - 1
            kappa[i] = get_curvature(track, agent.predicted_s[i + 1, 1])
        end
        
        if size(agent.predicted_s, 2) == 6
            kappa[end] = get_curvature(track, agent.predicted_s[end, 1] +
                                       dt * agent.predicted_s[end, 5])
        else
            kappa[end] = get_curvature(track, agent.predicted_s[end, 1] +
                                       dt * agent.predicted_s[end, 4])
        end
        =#
    end

    # z_ref1 = cat(2, zeros(N + 1, 3), v_ref * ones(N + 1, 1))
    # z_ref1 = [0.0 ey_ref 0.0 v_ref] .* ones(N + 1, 4)
    z_ref1 = reference .* ones(N + 1, 4)

    # z_ref2 = cat(2, zeros(mpcParams_pF.N + 1, 1), 0.2 * ones(mpcParams_pF.N+1,1),zeros(mpcParams_pF.N+1,1),v_ref*ones(mpcParams_pF.N+1,1))
    # z_ref3 = cat(2, zeros(mpcParams_pF.N + 1, 1), - 0.1 * ones(mpcParams_pF.N+1,1),zeros(mpcParams_pF.N+1,1),v_ref*ones(mpcParams_pF.N+1,1))

    sol_status::Symbol
    sol_u::Array{Float64,2}
    sol_z::Array{Float64,2}

    # Update current initial condition, curvature and previous input
    setvalue(mdl.z0, zCurr)
    setvalue(mdl.uPrev, uPrev)
    # println("Kappa: ", kappa)
    setvalue(mdl.curvature, kappa)  # Set approximate curvature

    setvalue(mdl.z_Ref, z_ref1)

    #=
    if lapStatus.currentLap == 1
        setvalue(mdl.z_Ref,z_ref1)
    elseif lapStatus.currentLap == 2
        setvalue(mdl.z_Ref,z_ref1)
    elseif lapStatus.currentLap == 3
        setvalue(mdl.z_Ref,z_ref1)
    end
    =#

    # Solve Problem and return solution
    sol_status = solve(mdl.mdl)
    sol_u = getvalue(mdl.u_Ol)
    sol_z = getvalue(mdl.z_Ol)

    curvature_values = zeros(N + 1)
    if POLYNOMIAL_CURVATURE
        println("Coefficents after: ", getvalue(mdl.curvature))
        polynom = Poly(getvalue(mdl.curvature))
        println(polynom)
        for i = 1 : N + 1
            curvature_values[i] = polyval(polynom, sol_z[i, 1])
        end
        println("Curvature: ", curvature_values)
    end

    # mpcSol.a_x = sol_u[1, 1]
    # mpcSol.d_f = sol_u[1, 2]
    # mpcSol.u = sol_u
    # mpcSol.z = sol_z
    # mpcSol.solverStatus = sol_status
    # mpcSol.cost = zeros(6)

    backward_mapping = [1, 2, 3, 4]
    optimizer.solution_inputs = sol_u
    optimizer.solution_states_s = sol_z[:, backward_mapping]

    publish_prediction(optimizer)
    publish_prediction_for_opponent_pf(mdl, optimizer, agent)

    #mpcSol.cost = [getvalue(mdl.costZ),0,0,getvalue(mdl.derivCost),getvalue(mdl.controlCost),0]

    # Print information
    # println("--------------- MPC PF START -----------------------------------------------")
    # println("z0             = $(zCurr')")
    println("Solved, status = $sol_status")
    # println("Predict. to s  = $(sol_z[end,1])")
    # #println("uPrev          = $(uPrev)")
    # println("--------------- MPC PF END ------------------------------------------------")
    nothing
end


# function solveMpcProblem_convhull(m::MpcModel_convhull,mpcSol::MpcSol,mpcCoeff::MpcCoeff,mpcParams::MpcParams,trackCoeff::TrackCoeff,lapStatus::LapStatus,
#                                  posInfo::PosInfo,modelParams::ModelParams,zCurr::Array{Float64},uPrev::Array{Float64},selectedStates::SelectedStates)
function solveMpcProblem_convhull(m::MpcModel_convhull, optimizer::Optimizer, 
                                  agent::Agent, track::Track)
 # Load Parameters
    sol_status::Symbol
    sol_u::Array{Float64,2}
    sol_z::Array{Float64,2}

    N = size(agent.optimal_inputs, 1) 
    kappa = zeros(N)
    dt = agent.dt

    # selStates = selectedStates.selStates::Array{Float64,2}
    # statesCost = selectedStates.statesCost::Array{Float64,1}

    # [s, e_y, e_psi, psi_dot, v_x, v_y] --> [v_x, v_y, psi_dot, e_psi, e_y, s]
    mapping = [5, 6, 4, 3, 2, 1]
    selected_states = agent.selected_states_s[:, mapping]
    selected_cost = agent.selected_states_cost

    iteration = agent.current_iteration
    current_s = agent.states_s[iteration, :]
    s = current_s[1]
    e_y = current_s[2]
    e_psi = current_s[3]
    psi_dot = current_s[4]
    v_x = current_s[5]
    v_y = current_s[6]

    try
        agent.acc = (getvalue(m.z_Ol))[2, 7]
    catch
        println("solution states have wrong size")
    end

    # println("INITIAL STATE: ", current_s)

    acc0 = agent.acc 
    # zCurr = [v_x, v_y, psi_dot, e_psi, e_y, s, acc0]
    zCurr = [v_x, v_y, psi_dot, e_psi, e_y, s]

    u_length = 10
    u_prev = zeros(u_length, 2)

    #=
    lap = agent.current_lap
    indeces = num_considered_states + 1 + iteration - 1 : - 1 : iteration - u_length + num_considered_states + 1
    u_prev = agent.previous_inputs[lap, indeces, :] 
    =#

    if iteration <= u_length
        u_prev[1 : iteration - 1, :] = agent.inputs[iteration - 1 : - 1 : 1, :] 
        current_lap = agent.current_lap
        if current_lap > 1
            previous_lap = current_lap - 1 + NUM_LOADED_LAPS
            needed_iter = agent.iterations_needed[previous_lap]
            indeces = needed_iter : - 1 : needed_iter - (u_length - iteration)
            u_prev[iteration : end, :] = squeeze(agent.previous_inputs[previous_lap, indeces, :], 1)
        end
    else
        u_prev = agent.inputs[iteration - 1 : - 1 : iteration - u_length, :]
    end   

    uPrev = u_prev

    if POLYNOMIAL_CURVATURE
        order = 5
        n_prev = 20
        n_after = 40
        total_num = n_prev + 1 + n_after
        num_coords = size(track.s_coord, 1)
        track_index = findmin(abs(track.s_coord - s))[2]
        println("Closest s: ", track.s_coord[track_index])
        s_data = zeros(total_num)
        kappa_data = zeros(total_num)

        if track_index - n_prev < 1
            difference = abs(track_index - n_prev)
            s_part_1 = track.s_coord[end - difference : end] - track.total_length
            s_part_2 = track.s_coord[1 : total_num - difference - 1]
            s_data = [s_part_1; s_part_2]
            kappa_part_1 = track.curvature[end - difference : end]
            kappa_part_2 = track.curvature[1 : total_num - difference - 1]
            kappa_data = [kappa_part_1; kappa_part_2]
        elseif track_index + n_after > num_coords
            difference = track_index + n_after - num_coords
            s_part_1 = track.s_coord[track_index - n_prev : end]
            s_part_2 = track.s_coord[1 : difference] + track.total_length
            s_data = [s_part_1; s_part_2]
            kappa_part_1 = track.curvature[track_index - n_prev : end]
            kappa_part_2 = track.curvature[1 : difference]
            kappa_data = [kappa_part_1; kappa_part_2]
        else 
            s_data = track.s_coord[track_index - n_prev : track_index + n_after]
            kappa_data = track.curvature[track_index - n_prev : track_index + n_after]
        end

        curvature_approx = polyfit(s_data, kappa_data, order)
        indeces = collect(track_index - n_prev : track_index + n_prev)
        kappa = coeffs(curvature_approx)
        # println("coefficents: ", kappa)
        
        #=
        app_curvature = zeros(indeces, Float64)
        counter = 1
        for i = track_index - n_prev : track_index + n_prev
            println("i: ", i)
            evaluated_poly = polyval(curvature_approx, track.s_coord[i])
            println("app curvautre: ", typeof(app_curvature[counter]))
            println(typeof(evaluated_poly))
            app_curvature[counter] = evaluated_poly
            println("Error: ", abs(evaluated_poly - track.curvature[i]))
            counter += 1
        end
        =#

        #=
        println("CURVATURE APPRXOIMATION: ", kappa)
        figure("test")
        plot(track.s_coord[indeces], track.curvature[indeces], "ro")
        plot(track.s_coord[indeces], app_curvature, "bo")
        plt[:show]()
        =#
    else
        for i = 1 : N
            kappa[i] = get_curvature(track, agent.predicted_s[i + 1, 1])
        end

        #=
        if size(agent.predicted_s, 2) == 6
            kappa[end] = get_curvature(track, agent.predicted_s[end, 1] +
                                       dt * agent.predicted_s[end, 5])
        else
            kappa[end] = get_curvature(track, agent.predicted_s[end, 1] +
                                       dt * agent.predicted_s[end, 4])
        end
        =#

        #=
        if size(agent.predicted_s, 2) == 6
            kappa[end] = get_curvature(track, agent.predicted_s[N + 1, 1])
        else
            kappa[end] = get_curvature(track, agent.predicted_s[N + 1, 1])
        end
        =#


        #=
        # println("Prediction before solving: ", agent.predicted_s)
        for i = 1 : N - 2
            kappa[i] = get_curvature(track, agent.predicted_s[i + 2, 1])
        end
        if size(agent.predicted_s, 2) == 6
            delta_s = dt * agent.predicted_s[end, 5]
            kappa[end - 1] = get_curvature(track, agent.predicted_s[end, 1] +
                                            delta_s)
            kappa[end] = get_curvature(track, agent.predicted_s[end, 1] +
                                             2 * delta_s)
        else
            delta_s = dt * agent.predicted_s[end, 4]
            kappa[end - 1] = get_curvature(track, agent.predicted_s[end, 1] +
                                           delta_s)
            kappa[end] = get_curvature(track, agent.predicted_s[end, 1] +
                                              2 * delta_s)
        end
        =#
    end

    # println("Propagated curvature: ", kappa)

    # Update current initial condition, curvature and System ID coefficients
    setvalue(m.z0, zCurr)
    setvalue(m.uPrev, uPrev)
    setvalue(m.c_Vx, agent.theta_vx)            # System ID coefficients
    setvalue(m.c_Vy, agent.theta_vy)
    setvalue(m.c_Psi, agent.theta_psi_dot)
    setvalue(m.curvature, kappa)       # Track curvature
    setvalue(m.selStates, selected_states)
    setvalue(m.statesCost, selected_cost)

     # Solve Problem and return solution
    sol_status = solve(m.mdl)
    sol_u = getvalue(m.u_Ol)
    sol_z = getvalue(m.z_Ol)

    curvature_values = zeros(N + 1)
    if POLYNOMIAL_CURVATURE
        println("Coefficents after: ", getvalue(m.curvature))
        polynom = Poly(getvalue(m.curvature))
        for i = 1 : N + 1
            curvature_values[i] = polyval(polynom, sol_z[i, 1])
        end
        println("Curvature: ", curvature_values)
    end

    backward_mapping = [6, 5, 4, 3, 1, 2]
    optimizer.solution_inputs = sol_u
    optimizer.solution_states_s = sol_z[:, backward_mapping]

    # export data
    # mpcSol.a_x = sol_u[1,1]
    # mpcSol.d_f = sol_u[1,2]
    # mpcSol.u   = sol_u
    # mpcSol.z   = sol_z
    #mpcSol.eps_alpha = getvalue(m.eps_alpha)
    # mpcSol.solverStatus = sol_status
    # mpcSol.cost = zeros(6)
    # mpcSol.cost = [0,getvalue(m.terminalCost),getvalue(m.controlCost),getvalue(m.derivCost),0,getvalue(m.laneCost)]

    # mpcSol.costSlack = zeros(6)
    # mpcSol.costSlack = [getvalue(m.slackVx),getvalue(m.slackVy),getvalue(m.slackPsidot),getvalue(m.slackEpsi),getvalue(m.slackEy),getvalue(m.slackS)]
    # println("slack values: ", [getvalue(m.slackVx), getvalue(m.slackVy), 
    #                           getvalue(m.slackPsidot), getvalue(m.slackEpsi), 
    #                           getvalue(m.slackEy), getvalue(m.slackS)])

    println(size(getvalue(m.alpha)))
    println(size(getvalue(m.selStates)))
    terminal_state = sum(getvalue(m.alpha) .* getvalue(m.selStates), 1)
    # println("terminal state:  ", terminal_state[backward_mapping])

    println("Solved, status = $sol_status")

    publish_prediction(optimizer)
    publish_prediction_for_opponent(m, optimizer, agent)

    nothing
end


function solveMpcProblem_obstacle(m::MpcModel_obstacle, optimizer::Optimizer, 
                                  agent::Agent, track::Track)
 # Load Parameters
    sol_status::Symbol
    sol_u::Array{Float64,2}
    sol_z::Array{Float64,2}

    N = size(agent.optimal_inputs, 1) 
    kappa = zeros(N)
    dt = agent.dt

    # selStates = selectedStates.selStates::Array{Float64,2}
    # statesCost = selectedStates.statesCost::Array{Float64,1}

    # [s, e_y, e_psi, psi_dot, v_x, v_y] --> [v_x, v_y, psi_dot, e_psi, e_y, s]
    mapping = [5, 6, 4, 3, 2, 1]
    selected_states = agent.selected_states_s[:, mapping]
    selected_cost = agent.selected_states_cost

    iteration = agent.current_iteration
    current_s = agent.states_s[iteration, :]
    s = current_s[1]
    e_y = current_s[2]
    e_psi = current_s[3]
    psi_dot = current_s[4]
    v_x = current_s[5]
    v_y = current_s[6]

    # println("INITIAL STATE: ", current_s)
    zCurr = [v_x, v_y, psi_dot, e_psi, e_y, s]

    u_length = 10
    u_prev = zeros(u_length, 2)

    if iteration <= u_length
        u_prev[1 : iteration - 1, :] = agent.inputs[iteration - 1 : - 1 : 1, :] 
        current_lap = agent.current_lap
        if current_lap > 1
            previous_lap = current_lap - 1 + NUM_LOADED_LAPS
            needed_iter = agent.iterations_needed[previous_lap]
            indeces = needed_iter : - 1 : needed_iter - (u_length - iteration)
            u_prev[iteration : end, :] = squeeze(agent.previous_inputs[previous_lap, indeces, :], 1)
        end
    else
        u_prev = agent.inputs[iteration - 1 : - 1 : iteration - u_length, :]
    end   

    uPrev = u_prev

    for i = 1 : N
        kappa[i] = get_curvature(track, agent.predicted_s[i + 1, 1])
    end

    # Update current initial condition, curvature and System ID coefficients
    setvalue(m.z0, zCurr)
    setvalue(m.uPrev, uPrev)
    setvalue(m.c_Vx, agent.theta_vx)            # System ID coefficients
    setvalue(m.c_Vy, agent.theta_vy)
    setvalue(m.c_Psi, agent.theta_psi_dot)
    setvalue(m.curvature, kappa)       # Track curvature
    setvalue(m.selStates, selected_states)
    setvalue(m.statesCost, selected_cost)

    setvalue(m.weights_obstacles, agent.weights_states)
    # println("ADV PREDICTIONS: ", optimizer.adv_predictions_s)

    if !agent.leading && s > optimizer.adv_predictions_s[1, 1]
        optimizer.adv_predictions_s[:, 1] += track.total_length
    elseif agent.leading && s < optimizer.adv_predictions_s[1, 1]
        optimizer.adv_predictions_s[:, 1] -= track.total_length
    end

    setvalue(m.adv_states_s, optimizer.adv_predictions_s)
    setvalue(m.current_lap, agent.current_lap)
    setvalue(m.adv_lap, optimizer.adv_current_lap)

     # Solve Problem and return solution
    sol_status = solve(m.mdl)
    sol_u = getvalue(m.u_Ol)
    sol_z = getvalue(m.z_Ol)

    backward_mapping = [6, 5, 4, 3, 1, 2]
    optimizer.solution_inputs = sol_u
    optimizer.solution_states_s = sol_z[:, backward_mapping]

    # export data
    # mpcSol.a_x = sol_u[1,1]
    # mpcSol.d_f = sol_u[1,2]
    # mpcSol.u   = sol_u
    # mpcSol.z   = sol_z
    #mpcSol.eps_alpha = getvalue(m.eps_alpha)
    # mpcSol.solverStatus = sol_status
    # mpcSol.cost = zeros(6)
    # mpcSol.cost = [0,getvalue(m.terminalCost),getvalue(m.controlCost),getvalue(m.derivCost),0,getvalue(m.laneCost)]

    # mpcSol.costSlack = zeros(6)
    # mpcSol.costSlack = [getvalue(m.slackVx),getvalue(m.slackVy),getvalue(m.slackPsidot),getvalue(m.slackEpsi),getvalue(m.slackEy),getvalue(m.slackS)]
    # println("slack values: ", [getvalue(m.slackVx), getvalue(m.slackVy), 
    #                           getvalue(m.slackPsidot), getvalue(m.slackEpsi), 
    #                           getvalue(m.slackEy), getvalue(m.slackS)])

    println(size(getvalue(m.alpha)))
    println(size(getvalue(m.selStates)))
    terminal_state = sum(getvalue(m.alpha) .* getvalue(m.selStates), 1)
    # println("terminal state:  ", terminal_state[backward_mapping])

    println("Solved, status = $sol_status")

    println("obstacle cost: ", getvalue(m.obstacle_cost))

    println("test: ", ((optimizer.adv_predictions_s[:, 1] - optimizer.solution_states_s[:, 1]) / 0.25).^2 + 
                      ((optimizer.adv_predictions_s[:, 2] - optimizer.solution_states_s[:, 2]) / 0.25).^2 - 1)

    #=
    if sol_status == :Error
        exit()
    end
    =#

    publish_prediction(optimizer)
    publish_prediction_for_opponent_obs(m, optimizer, agent)

    nothing
end


function solveMpcProblem_obstacle(m::MpcModel_obstacle, optimizer::Optimizer, 
                                  agent::Agent, track::Track, leading::Bool)
    if leading 
        obstacle_weight = 1.0 * 0.1
        safety_weight = 1.0
    else 
        obstacle_weight = 0.5
        safety_weight = 0.5
    end

    setvalue(m.weights_obstacle, obstacle_weight)
    setvalue(m.safety_weight, safety_weight)

    solveMpcProblem_obstacle(m, optimizer, agent, track)
end

function publish_prediction_for_opponent_pf(m::MpcModel_pF, 
                                         optimizer::Optimizer, agent::Agent)
    propagated_prediction = zeros(optimizer.solution_states_s)
    propagated_prediction[1 : end - 1, :] = optimizer.solution_states_s[2 : end, :]
    propagated_prediction[end, :] = optimizer.solution_states_s[end, :]

    optimizer.prediction_for_opp.header.stamp = get_rostime()
    optimizer.prediction_for_opp.s = propagated_prediction[:, 1]
    optimizer.prediction_for_opp.ey = propagated_prediction[:, 2]
    optimizer.prediction_for_opp.epsi = propagated_prediction[:, 3]
    optimizer.prediction_for_opp.psidot = zeros(propagated_prediction[:, 4])
    optimizer.prediction_for_opp.vx = propagated_prediction[:, 4]
    optimizer.prediction_for_opp.vy = zeros(propagated_prediction[:, 4])
    optimizer.prediction_for_opp.current_lap = optimizer.agent.current_lap

    publish(optimizer.prediction_opponent_pub, optimizer.prediction_for_opp)
end


function publish_prediction_for_opponent(m::MpcModel_convhull, 
                                         optimizer::Optimizer, agent::Agent)
    # Publish already propagated prediction for the opponent
    selected_laps = agent.selected_laps
    indeces = agent.closest_indeces
    propagated_states = zeros(agent.selected_states_s)

    index = 1
    for i = 1 : size(selected_laps, 1)
        propagated_states[index : index + NUM_HORIZONS * HORIZON - 1, :] = squeeze(agent.trajectories_s[selected_laps[i], indeces[i] : indeces[i] + NUM_HORIZONS * HORIZON - 1, :], 1)
        index += NUM_HORIZONS * HORIZON
    end

    propagated_state = sum(getvalue(m.alpha) .* propagated_states, 1)
    propagated_prediction = zeros(optimizer.solution_states_s)
    propagated_prediction[1 : end - 1, :] = optimizer.solution_states_s[2 : end, :]
    
    threshold = 0.1

    if abs(propagated_state[1] - optimizer.solution_states_s[end, 1]) < abs(optimizer.solution_states_s[end - 1, 1] - optimizer.solution_states_s[end, 1]) + threshold
        propagated_prediction[end, :] = propagated_state
    else
        propagated_prediction[end, :] = optimizer.solution_states_s[end, :]
    end

    optimizer.prediction_for_opp.header.stamp = get_rostime()
    optimizer.prediction_for_opp.s = propagated_prediction[:, 1]
    optimizer.prediction_for_opp.ey = propagated_prediction[:, 2]
    optimizer.prediction_for_opp.epsi = propagated_prediction[:, 3]
    optimizer.prediction_for_opp.psidot = propagated_prediction[:, 4]
    optimizer.prediction_for_opp.vx = propagated_prediction[:, 5]
    optimizer.prediction_for_opp.vy = propagated_prediction[:, 6]
    optimizer.prediction_for_opp.current_lap = optimizer.agent.current_lap

    publish(optimizer.prediction_opponent_pub, optimizer.prediction_for_opp)
end

function publish_prediction_for_opponent_obs(m::MpcModel_obstacle, 
                                         optimizer::Optimizer, agent::Agent)
    # Publish already propagated prediction for the opponent
    selected_laps = agent.selected_laps
    indeces = agent.closest_indeces
    propagated_states = zeros(agent.selected_states_s)

    index = 1
    for i = 1 : size(selected_laps, 1)
        propagated_states[index : index + NUM_HORIZONS * HORIZON - 1, :] = squeeze(agent.trajectories_s[selected_laps[i], indeces[i] : indeces[i] + NUM_HORIZONS * HORIZON - 1, :], 1)
        index += NUM_HORIZONS * HORIZON
    end

    propagated_state = sum(getvalue(m.alpha) .* propagated_states, 1)
    propagated_prediction = zeros(optimizer.solution_states_s)
    propagated_prediction[1 : end - 1, :] = optimizer.solution_states_s[2 : end, :]
    
    threshold = 0.1

    if abs(propagated_state[1] - optimizer.solution_states_s[end, 1]) < abs(optimizer.solution_states_s[end - 1, 1] - optimizer.solution_states_s[end, 1]) + threshold
        propagated_prediction[end, :] = propagated_state
    else
        propagated_prediction[end, :] = optimizer.solution_states_s[end, :]
    end

    optimizer.prediction_for_opp.header.stamp = get_rostime()
    optimizer.prediction_for_opp.s = propagated_prediction[:, 1]
    optimizer.prediction_for_opp.ey = propagated_prediction[:, 2]
    optimizer.prediction_for_opp.epsi = propagated_prediction[:, 3]
    optimizer.prediction_for_opp.psidot = propagated_prediction[:, 4]
    optimizer.prediction_for_opp.vx = propagated_prediction[:, 5]
    optimizer.prediction_for_opp.vy = propagated_prediction[:, 6]
    optimizer.prediction_for_opp.current_lap = optimizer.agent.current_lap

    publish(optimizer.prediction_opponent_pub, optimizer.prediction_for_opp)
end