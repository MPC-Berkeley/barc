# THERE ARE TWO DIFFERENT CORRESPONDING mpc_solving FUNCTIONS FOR THIS MODEL
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
    df_his::Array{JuMP.NonlinearParameter,1}

    function MpcModel_pF(mpcParams_pF::MpcParams,modelParams::ModelParams)
        m = new()
        # Model parameters
        dt          = modelParams.dt
        L_a         = modelParams.l_A
        L_b         = modelParams.l_B
        # u_lb        = mpcParams_pF.u_lb
        # u_ub        = mpcParams_pF.u_ub
        # z_lb        = mpcParams_pF.z_lb
        # z_ub        = mpcParams_pF.z_ub
        u_lb = [-1    -18/180*pi]
        u_ub = [ 2     18/180*pi]
        z_lb = [-Inf -Inf -Inf -0.5] # 1.s 2.ey 3.epsi 4.v
        z_ub = [ Inf  Inf  Inf  2.5] # 1.s 2.ey 3.epsi 4.v

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
        @NLparameter(mdl, df_his[1:3] == 0)
        @NLparameter(mdl, zPrev[1:N+1,1:4]==0)
        @NLparameter(mdl, c[1:N+1]==0)

        # System dynamics
        @NLconstraint(mdl, [i=1:4], z_Ol[1,i] == z0[i])         # initial condition
        # # THIS IS FOR SIMULATION
        # @NLconstraint(mdl, u_Ol[1,2] == uPrev[2,2]) # steering delay is 1 for simulation and 3 for experiment
        # THIS IS FOR EXPERIEMNT
        @NLconstraint(mdl, [i=1:3], u_Ol[i,2] == df_his[i]) # steering delay is 1 for simulation and 3 for experiment

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
        m.df_his = df_his
        m.derivCost=derivCost; m.costZ=costZ; m.controlCost=controlCost
        return m
    end
end

type MpcModel_convhull_dyn_iden

    mdl::JuMP.Model

    z0::Array{JuMP.NonlinearParameter,1}
    # coeff::Array{JuMP.NonlinearParameter,1}
    selStates::Array{JuMP.NonlinearParameter,2}
    statesCost::Array{JuMP.NonlinearParameter,1}
    c_Vx::Array{JuMP.NonlinearParameter,2}
    c_Vy::Array{JuMP.NonlinearParameter,2}
    c_Psi::Array{JuMP.NonlinearParameter,2}
    uPrev::Array{JuMP.NonlinearParameter,2}
    df_his::Array{JuMP.NonlinearParameter,1}

    eps_lane::Array{JuMP.Variable,1}
    alpha::Array{JuMP.Variable,1}
    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}

    dsdt::Array{JuMP.NonlinearExpression,1}
    c::Array{JuMP.NonlinearParameter,1}

    derivCost::JuMP.NonlinearExpression
    # controlCost::JuMP.NonlinearExpression
    laneCost::JuMP.NonlinearExpression
    terminalCost::JuMP.NonlinearExpression
    # slackVx::JuMP.NonlinearExpression
    # slackVy::JuMP.NonlinearExpression
    # slackPsidot::JuMP.NonlinearExpression
    # slackEpsi::JuMP.NonlinearExpression
    # slackEy::JuMP.NonlinearExpression
    # slackS::JuMP.NonlinearExpression

    function MpcModel_convhull_dyn_iden(mpcParams::MpcParams,modelParams::ModelParams)
        m = new()
        n_state=6; n_input=2
        #### Initialize parameters
        dt         = modelParams.dt              # time step
        L_a        = modelParams.l_A             # distance from CoM of the car to the front wheels
        L_b        = modelParams.l_B             # distance from CoM of the car to the rear wheels
        # u_lb       = mpcParams.u_lb              # lower bounds for the control inputs
        # u_ub       = mpcParams.u_ub              # upper bounds for the control inputs
        # z_lb       = mpcParams.z_lb              # lower bounds for the states
        # z_ub       = mpcParams.z_ub              # upper bounds for the states
        u_lb = [ -0.5    -18/180*pi]
        u_ub = [    2     18/180*pi]
        z_lb = [-Inf -0.6 -pi    0 -1 -2*pi] # 1.s 2.ey 3.epsi 4.vx 5.vy 6.psi_dot
        z_ub = [ Inf  0.6  pi  2.5  1  2*pi] # 1.s 2.ey 3.epsi 4.vx 5.vy 6.psi_dot

        ey_max      = 0.6/2           # bound for the state ey (distance from the center track). It is set as half of the width of the track for obvious reasons

        N          = mpcParams.N                           # Prediction horizon
        QderivZ    = mpcParams.QderivZ::Array{Float64,1}   # weights for the derivative cost on the states
        QderivU    = mpcParams.QderivU::Array{Float64,1}   # weights for the derivative cost on the control inputs
        Q_term_cost = mpcParams.Q_term_cost
        R          = mpcParams.R::Array{Float64,1}         # weights on the control inputs
        Q          = mpcParams.Q::Array{Float64,1}         # weights on the states for path following
        Q_lane     = mpcParams.Q_lane::Float64             # weight on the soft constraint on the lane
        Q_slack    = mpcParams.Q_slack

        Np         = 10
        Nl         = 2

        mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.09)) #,linear_solver="ma27"))#,check_derivatives_for_naninf="yes"))#,linear_solver="ma57",print_user_options="yes"))
        # 
        @variable( mdl, z_Ol[1:(N+1),1:n_state])
        @variable( mdl, u_Ol[1:N,1:n_input])
        @variable( mdl, eps_lane[1:N] >= 0)   # eps for soft lane constraints
        @variable( mdl, alpha[1:Nl*Np] >= 0)    # coefficients of the convex hull

        for j=1:N
            for i=1:n_input
                setlowerbound(u_Ol[j,i], u_lb[i])
                setupperbound(u_Ol[j,i], u_ub[i])
            end
            for i=1:n_state
                setlowerbound(z_Ol[j+1,i], z_lb[i])
                setupperbound(z_Ol[j+1,i], z_ub[i])
            end
        end
        for i=1:n_state
            setlowerbound(z_Ol[N+1,i], z_lb[i])
            setupperbound(z_Ol[N+1,i], z_ub[i])
        end

        @NLparameter(mdl, z0[i=1:n_state] == 0)
        @NLparameter(mdl, c[1:N] == 0)
        @NLparameter(mdl, c_Vx[1:N,1:3]  == 0)    # system identification parameters
        @NLparameter(mdl, c_Vy[1:N,1:4]  == 0)    # system identification parameters
        @NLparameter(mdl, c_Psi[1:N,1:3] == 0)    # system identification parameters
        @NLparameter(mdl, uPrev[1:N,1:2] == 0)
        @NLparameter(mdl, df_his[1:3] == 0)
        @NLparameter(mdl, selStates[1:Nl*Np,1:n_state] == 0)   # states from the previous trajectories selected in "convhullStates"
        @NLparameter(mdl, statesCost[1:Nl*Np] == 0)      # costs of the states selected in "convhullStates"

        @NLconstraint(mdl, [i=1:n_state], z_Ol[1,i] == z0[i])
        # THIS INPUT CONSTRAINT IS FOR SYSTEM DELAY
        @NLconstraint(mdl, u_Ol[1,1] == uPrev[2,1]) # acceleration delay is 1
        # # THIS IS FOR SIMULATION
        # @NLconstraint(mdl, u_Ol[1,2] == uPrev[2,2]) # steering delay is 1 for simulation and 3 for experiment
        # THIS IS FOR EXPERIEMNT
        @NLconstraint(mdl, [i=1:3], u_Ol[i,2] == df_his[i]) # steering delay is 1 for simulation and 3 for experiment

        @NLconstraint(mdl, [i=2:N+1], z_Ol[i,2] <= ey_max + eps_lane[i-1])
        @NLconstraint(mdl, [i=2:N+1], z_Ol[i,2] >= -ey_max - eps_lane[i-1])
        @NLconstraint(mdl, sum{alpha[i],i=1:Nl*Np} == 1)

        @NLexpression(mdl, dsdt[i = 1:N], (z_Ol[i,4]*cos(z_Ol[i,3]) - z_Ol[i,5]*sin(z_Ol[i,3]))/(1-z_Ol[i,2]*c[i]))

        # for i=1:n_state
        #     @NLconstraint(mdl, z_Ol[N+1,i] == sum(alpha[j]*selStates[j,i] for j=1:Nl*Np))
        # end

        # System dynamics
        for i=1:N
            @NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + dt * dsdt[i])                                                # s
            @NLconstraint(mdl, z_Ol[i+1,2]  == z_Ol[i,2] + dt * (z_Ol[i,4]*sin(z_Ol[i,3]) + z_Ol[i,5]*cos(z_Ol[i,3])))  # eY
            @NLconstraint(mdl, z_Ol[i+1,3]  == z_Ol[i,3] + dt * (z_Ol[i,6]-dsdt[i]*c[i]))                               # ePsi
            @NLconstraint(mdl, z_Ol[i+1,4]  == z_Ol[i,4] + c_Vx[i,1]*z_Ol[i,5]*z_Ol[i,6] + c_Vx[i,2]*z_Ol[i,4] + c_Vx[i,3]*u_Ol[i,1])                                         # vx
            # @NLconstraint(mdl, z_Ol[i+1,4]  == z_Ol[i,4] + 0.1*z_Ol[i,5]*z_Ol[i,6] + c_Vx[i,2]*z_Ol[i,4] + c_Vx[i,3]*u_Ol[i,1])                                         # vx
            @NLconstraint(mdl, z_Ol[i+1,5]  == z_Ol[i,5] + c_Vy[i,1]*z_Ol[i,5]/z_Ol[i,4] + c_Vy[i,2]*z_Ol[i,4]*z_Ol[i,6] + c_Vy[i,3]*z_Ol[i,6]/z_Ol[i,4] + c_Vy[i,4]*u_Ol[i,2]) # vy
            @NLconstraint(mdl, z_Ol[i+1,6]  == z_Ol[i,6] + c_Psi[i,1]*z_Ol[i,6]/z_Ol[i,4] + c_Psi[i,2]*z_Ol[i,5]/z_Ol[i,4] + c_Psi[i,3]*u_Ol[i,2])                            # psiDot
        end

        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] <= 0.12)
        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] >= -0.12)
        for i=1:N-1 # hard constraints on u, which means this confition is really serious
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] <= 0.12)
            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] >= -0.12)
        end
        # Cost functions
        @NLexpression(mdl, derivCost, sum{QderivZ[j]*(sum{(z_Ol[i,j]-z_Ol[i+1,j])^2 ,i=1:N}), j=1:n_state} +
                                      sum{QderivU[j]*(sum{(u_Ol[i,j]-u_Ol[i+1,j])^2 ,i=1:N-1} + (uPrev[1,j]-u_Ol[1,j])^2) ,j=1:n_input} )
        # @NLexpression(mdl, controlCost, sum(R[j]*sum((u_Ol[i,1])^2 for i=1:N) for j=1:2))
        @NLexpression(mdl, laneCost, Q_lane*sum{10.0*eps_lane[i]+50.0*eps_lane[i]^2 , i=1:N}) # original data for lane cost 10.0*eps_lane[i]+50.0*eps_lane[i]^2
        @NLexpression(mdl, terminalCost , Q_term_cost*sum{alpha[i]*statesCost[i] , i=1:Nl*Np})
        # Slack cost on vx
        #----------------------------------
        @NLexpression(mdl, slackVx, (z_Ol[N+1,4] - sum{alpha[j]*selStates[j,4] , j=1:Nl*Np})^2)

        # Slack cost on vy
        #----------------------------------
        @NLexpression(mdl, slackVy, (z_Ol[N+1,5] - sum{alpha[j]*selStates[j,5] , j=1:Nl*Np})^2)

        # Slack cost on Psi dot
        #----------------------------------
        @NLexpression(mdl, slackPsidot, (z_Ol[N+1,6] - sum{alpha[j]*selStates[j,6] ,j=1:Nl*Np})^2)

        # Slack cost on ePsi
        #----------------------------------
        @NLexpression(mdl, slackEpsi, (z_Ol[N+1,3] - sum{alpha[j]*selStates[j,3] , j=1:Nl*Np})^2)

        # Slack cost on ey
        #----------------------------------
        @NLexpression(mdl, slackEy, (z_Ol[N+1,2] - sum{alpha[j]*selStates[j,2] , j=1:Nl*Np})^2)

        # Slack cost on s
        #----------------------------------
        @NLexpression(mdl, slackS, (z_Ol[N+1,1] - sum{alpha[j]*selStates[j,1] , j=1:Nl*Np})^2)

        @NLobjective(mdl, Min, derivCost + laneCost +  terminalCost + Q_slack[1]*slackS + Q_slack[2]*slackEy + Q_slack[3]*slackEpsi + Q_slack[4]*slackVx + Q_slack[5]*slackVy + Q_slack[6]*slackPsidot) #+ controlCost

        m.mdl = mdl
        m.z0 = z0
        m.c = c
        m.z_Ol = z_Ol
        m.u_Ol = u_Ol
        m.c_Vx = c_Vx
        m.c_Vy = c_Vy
        m.c_Psi = c_Psi
        m.uPrev = uPrev
        m.df_his = df_his

        m.derivCost = derivCost
        # m.controlCost = controlCost
        m.laneCost = laneCost
        m.terminalCost= terminalCost # terminal cost
        m.selStates   = selStates    # selected states
        m.statesCost  = statesCost   # cost of the selected states
        m.alpha       = alpha        # parameters of the convex hull

        # m.slackVx     = slackVx
        # m.slackVy     = slackVy
        # m.slackPsidot = slackPsidot
        # m.slackEpsi   = slackEpsi
        # m.slackEy     = slackEy
        # m.slackS      = slackS
        return m
    end
end

type MpcModel_convhull_kin_linear

    mdl::JuMP.Model

    selStates::Array{JuMP.NonlinearParameter,2}
    statesCost::Array{JuMP.NonlinearParameter,1}
    uPrev::Array{JuMP.NonlinearParameter,2}
    zPrev::Array{JuMP.NonlinearParameter,2}

    A::Array{JuMP.NonlinearParameter,3}
    B::Array{JuMP.NonlinearParameter,3}

    eps_lane::Array{JuMP.Variable,1}
    alpha::Array{JuMP.Variable,1}
    z_Ol::Array{JuMP.Variable,2}
    u_Ol::Array{JuMP.Variable,2}
    z_linear::Array{JuMP.NonlinearParameter,2}
    u_linear::Array{JuMP.NonlinearParameter,2}

    derivCost
    laneCost
    terminalCost
    # slackS
    # slackEy
    # slackEpsi
    # slackV

    function MpcModel_convhull_kin_linear(mpcParams::MpcParams,modelParams::ModelParams)
        m = new(); n_state=4; n_input=2
        # A big difference in this model is that the first state vector is fixed variable, which is intialized individually
        # So the dimensions of value assignment need to match and be paied attention
        # IMPORTANT: decision variables here are delta state, which needs to be added back to the linearization point
        #### Initialize parameters
        dt         = modelParams.dt                # time step
        L_a        = modelParams.l_A               # distance from CoM of the car to the front wheels
        L_b        = modelParams.l_B               # distance from CoM of the car to the rear wheels
        # u_lb       = mpcParams.u_lb              # lower bounds for the control inputs
        # u_ub       = mpcParams.u_ub              # upper bounds for the control inputs
        # z_lb       = mpcParams.z_lb              # lower bounds for the states
        # z_ub       = mpcParams.z_ub              # upper bounds for the states
        u_lb = [-1    -18/180*pi]
        u_ub = [ 2     18/180*pi]
        z_lb = [-Inf -0.6 -pi -0.5] # 1.s 2.ey 3.epsi 4.v
        z_ub = [ Inf  0.6  pi  2.5] # 1.s 2.ey 3.epsi 4.v

        c_f        = modelParams.c_f

        ey_max      = 0.6/2                    # bound for the state ey (distance from the center track). It is set as half of the width of the track for obvious reasons

        N          = mpcParams.N                           # Prediction horizon
        QderivZ    = mpcParams.QderivZ::Array{Float64,1}   # weights for the derivative cost on the states
        QderivU    = mpcParams.QderivU::Array{Float64,1}   # weights for the derivative cost on the control inputs
        R          = mpcParams.R::Array{Float64,1}         # weights on the control inputs
        Q          = mpcParams.Q::Array{Float64,1}         # weights on the states for path following
        Q_term_cost= mpcParams.Q_term_cost::Float64        # weights on the terminal states
        Q_lane     = mpcParams.Q_lane::Float64             # weight on the soft constraint on the lane
        Q_vel      = mpcParams.Q_vel::Float64             # weight on the soft constraint for the max velocity
        Q_slack    = mpcParams.Q_slack

        Np         = 10              # how many states to select
        Nl         = 2               # how many previous laps to select

        mdl = Model(solver = IpoptSolver(print_level=0,linear_solver="ma27",max_cpu_time=0.09))#,check_derivatives_for_naninf="yes"))#,linear_solver="ma57",print_user_options="yes"))

        @NLparameter( mdl, zPrev[1:N+1,1:n_state] == 0)
        @NLparameter( mdl, uPrev[1:N,1:n_input] == 0)
        @NLparameter( mdl, z_linear[1:N+1,1:n_state] == 0) # reference variables, which need to be added back to delta decision variables
        @NLparameter( mdl, u_linear[1:N,1:n_input] == 0)
        @NLparameter( mdl, A[1:n_state,1:n_state,1:N]==0)
        @NLparameter( mdl, B[1:n_state,1:n_input,1:N]==0)
        @NLparameter( mdl, selStates[1:Nl*Np,1:n_state]==0)
        @NLparameter( mdl, statesCost[1:Nl*Np]==0)

        @variable( mdl, z_Ol[1:N,1:n_state])
        @variable( mdl, u_Ol[1:N,1:n_input])
        @variable( mdl, eps_lane[1:N] >= 0)   # eps for soft lane constraints
        @variable( mdl, alpha[1:Nl*Np] >= 0)  # coefficients of the convex hull

        @NLconstraint(mdl, [i=1:N], z_Ol[i,2] + z_linear[i+1,2] <= ey_max + eps_lane[i])
        @NLconstraint(mdl, [i=1:N], z_Ol[i,2] + z_linear[i+1,2] >= -ey_max - eps_lane[i])
        @NLconstraint(mdl, sum{alpha[i] , i=1:Nl*Np} == 1)

        # Variable boundary hard constraint on variables
        @NLconstraint(mdl, [i=1:N], z_Ol[i,4] + z_linear[i+1,4] <= z_ub[4])
        @NLconstraint(mdl, [i=1:N], z_Ol[i,4] + z_linear[i+1,4] >= z_lb[4])
        for j=1:n_input
            @NLconstraint(mdl, [i=1:N], u_Ol[i,j] + u_linear[i,j] >= u_lb[j])
            @NLconstraint(mdl, [i=1:N], u_Ol[i,j] + u_linear[i,j] <= u_ub[j])
        end

        # System dynamics
        for i=1:N-1
            for j=1:n_state
                @NLconstraint(mdl, z_Ol[i+1,j] == A[j,1,i+1]*z_Ol[i,1]+A[j,2,i+1]*z_Ol[i,2]+A[j,3,i+1]*z_Ol[i,3]+A[j,4,i+1]*z_Ol[i,4]+B[j,1,i+1]*u_Ol[i+1,1]+B[j,2,i+1]*u_Ol[i+1,2] )
            end
        end
        for j=1:n_state
            # @constraint(mdl, z_Ol[1,j] == dot(B[j,:,1],u_Ol[1,:]) )
            @NLconstraint(mdl, z_Ol[1,j] == B[j,1,1]*u_Ol[1,1]+B[j,2,1]*u_Ol[1,2] )
        end

        # for i=1:N-1
        #     @constraint(mdl, z_Ol[i+1,:] .== A[:,:,i+1]*z_Ol[i,:] + B[:,:,i+1]*u_Ol[i+1,:] )
        # end

        @NLconstraint(mdl, u_Ol[1,2]+u_linear[1,2]-uPrev[1,2] <= 0.12)
        @NLconstraint(mdl, u_Ol[1,2]+u_linear[1,2]-uPrev[1,2] >= -0.12)
        for i=1:N-1
            @NLconstraint(mdl, u_Ol[i+1,2]+u_linear[i+1,2]-(u_Ol[i,2]+u_linear[i,2]) <= 0.12)
            @NLconstraint(mdl, u_Ol[i+1,2]+u_linear[i+1,2]-(u_Ol[i,2]+u_linear[i,2]) >= -0.12)
        end

        # for i=1:n_state
        #     @constraint(mdl, z_Ol[N,i] + z_linear[N+1,i] == sum(alpha[j]*selStates[j,i] for j=1:Nl*Np))
        # end

        # Cost functions
        # Derivative cost
        @NLexpression(mdl, derivCost, sum{QderivZ[j]*(sum{(z_Ol[i,j]+z_linear[i+1,j]-z_Ol[i+1,j]-z_linear[i+2,j])^2 , i=1:N-1}+(zPrev[1,j]-z_Ol[1,j]-z_linear[2,j])^2) , j=1:n_state} +
                                    sum{QderivU[j]*(sum{(u_Ol[i,j]+u_linear[i  ,j]-u_Ol[i+1,j]-u_linear[i+1,j])^2 , i=1:N-1}+(uPrev[1,j]-u_Ol[1,j]-u_linear[1,j])^2) , j=1:n_input})
        # Lane cost (soft)
        @NLexpression(mdl, laneCost, Q_lane*sum{10.0*eps_lane[i]+50.0*eps_lane[i]^2 , i=1:N})
        # Terminal Cost
        @NLexpression(mdl, terminalCost , Q_term_cost*sum{alpha[i]*statesCost[i] , i=1:Nl*Np})
        ###### SLACK VARIABLES: ######
        # When due to the linear modeling in JuMP, there is no parameter as NLparameters,
        # which will cause the noolinear expression here
        ###### Slack cost on s ######
        @NLexpression(mdl, slackS, (z_Ol[N,1] + z_linear[N+1,1] - sum{alpha[j]*selStates[j,1] , j=1:Nl*Np})^2)
        # Slack cost on ey
        @NLexpression(mdl, slackEy, (z_Ol[N,2] + z_linear[N+1,2] - sum{alpha[j]*selStates[j,2] , j=1:Nl*Np})^2)
        # Slack cost on ePsi
        @NLexpression(mdl, slackEpsi, (z_Ol[N,3] + z_linear[N+1,3] - sum{alpha[j]*selStates[j,3] , j=1:Nl*Np})^2)
        # Slack cost on v
        @NLexpression(mdl, slackV, (z_Ol[N,4] + z_linear[N+1,4] - sum{alpha[j]*selStates[j,4] , j=1:Nl*Np})^2)

        @NLobjective(mdl, Min, derivCost + laneCost + terminalCost + Q_slack[1]*slackS + Q_slack[2]*slackEy + Q_slack[3]*slackEpsi + Q_slack[4]*slackV) #+ Q_slack[5]*slackEy + Q_slack[6]*slackS) #+ controlCost

        m.mdl = mdl
        m.z_Ol = z_Ol
        m.u_Ol = u_Ol
        m.z_linear = z_linear
        m.u_linear = u_linear
        m.zPrev = zPrev
        m.uPrev = uPrev

        m.derivCost = derivCost
        m.laneCost = laneCost
        m.terminalCost= terminalCost # terminal cost
        m.selStates   = selStates    # selected states
        m.statesCost  = statesCost   # cost of the selected states
        m.alpha       = alpha        # parameters of the convex hull
        m.eps_lane    = eps_lane

        m.A=A
        m.B=B

        # m.slackS      = slackS
        # m.slackEy     = slackEy
        # m.slackEpsi   = slackEpsi
        # m.slackV      = slackV

        return m
    end
end

