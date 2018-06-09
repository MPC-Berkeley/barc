module mpcModels
using RobotOS
using JuMP
using Ipopt
using Types
export MdlPf,MdlKin,MdlId,MdlKinLin,MdlDynLin
    type MdlPf
        # Fields here provides a channel for JuMP model to communicate with data outside of it
        mdl::JuMP.Model

        z0::Array{JuMP.NonlinearParameter,1}
        c::Array{JuMP.NonlinearParameter,1}
        z_Ref::Array{JuMP.NonlinearParameter,2}

        z_Ol::Array{JuMP.Variable,2}
        u_Ol::Array{JuMP.Variable,2}

        uPrev::Array{JuMP.NonlinearParameter,2}
        a_his::Array{JuMP.NonlinearParameter,1}
        df_his::Array{JuMP.NonlinearParameter,1}

        function MdlPf(agent::Agent)
            m = new()
            # Model parameters
            dt   = get_param("controller/dt")
            L_a  = get_param("L_a")
            L_b  = get_param("L_b")
            
            u_lb = [-1    -18/180*pi]
            u_ub = [ 2     18/180*pi]
            z_lb = [-Inf -Inf -Inf -0.5] # 1.s 2.ey 3.epsi 4.v
            z_ub = [ Inf  Inf  Inf  3.0] # 1.s 2.ey 3.epsi 4.v

            c_f     = get_param("controller/c_f")
            N       = get_param("controller/N")
            Q       = agent.mpcParams.Q
            R       = agent.mpcParams.R
            QderivZ = agent.mpcParams.QderivZ
            QderivU = agent.mpcParams.QderivU
            v_ref   = agent.mpcParams.vPf

            mdl = Model(solver = IpoptSolver(print_level=0,linear_solver="ma27",max_cpu_time=0.09))

            @variable( mdl, z_Ol[1:(N+1),1:4],  start = 0)
            @variable( mdl, u_Ol[1:N,1:2],      start = 0)

            for j=1:N
                for i=1:2
                    setlowerbound(u_Ol[j,i], u_lb[i])
                    setupperbound(u_Ol[j,i], u_ub[i])
                end
                setlowerbound(z_Ol[j+1,4], z_lb[4])
                setupperbound(z_Ol[j+1,4], z_ub[4])
            end

            # Nonlinear parameters initialization
            @NLparameter(mdl, z_Ref[1:N+1,1:4]==0)
            setvalue(z_Ref,hcat(zeros(N+1,3),v_ref*ones(N+1,1)))
            @NLparameter(mdl, z0[i=1:4]==0)
            @NLparameter(mdl, uPrev[1:N,1:2]==0)
            delay_df = get_param("controller/delay_df")
            if delay_df>0
                @NLparameter(mdl, df_his[1:delay_df] == 0)
                @NLconstraint(mdl, [i=1:delay_df], u_Ol[i,2] == df_his[i])
                m.df_his = df_his
            end
            delay_a = get_param("controller/delay_a")
            if delay_a>0
                @NLparameter(mdl, a_his[1:delay_a] == 0)
                @NLconstraint(mdl, [i=1:delay_a], u_Ol[i,1] == a_his[i])
                m.a_his = a_his
            end
            @NLparameter(mdl, c[1:N]==0)

            # System dynamics
            @NLconstraint(mdl, [i=1:4], z_Ol[1,i] == z0[i])
            

            for i=1:N
                # @NLexpression(mdl, bta[i],atan( L_a / (L_a + L_b) * tan(u_Ol[i,2])))
                @NLexpression(mdl, bta[i],  L_a/(L_a + L_b)*u_Ol[i,2])
                @NLexpression(mdl, dsdt[i], z_Ol[i,4]*cos(z_Ol[i,3]+bta[i])/(1-z_Ol[i,2]*c[i]))
                @NLconstraint(mdl, z_Ol[i+1,1] == z_Ol[i,1] + dt*dsdt[i]  ) 
                @NLconstraint(mdl, z_Ol[i+1,2] == z_Ol[i,2] + dt*z_Ol[i,4]*sin(z_Ol[i,3]+bta[i])  )
                @NLconstraint(mdl, z_Ol[i+1,3] == z_Ol[i,3] + dt*(z_Ol[i,4]/L_a*sin(bta[i])-dsdt[i]*c[i])  )
                @NLconstraint(mdl, z_Ol[i+1,4] == z_Ol[i,4] + dt*(u_Ol[i,1] - c_f*z_Ol[i,4]))
            end
            @NLexpression(mdl, derivCost, sum{QderivZ[j]*sum{(z_Ol[i,j]-z_Ol[i+1,j])^2,i=1:N},j=1:4} +
                                              QderivU[1]*sum{(u_Ol[i,1]-u_Ol[i+1,1])^2,i=delay_a+1:N-1} + 
                                              QderivU[2]*sum{(u_Ol[i,2]-u_Ol[i+1,2])^2,i=delay_df+1:N-1} + 
                                              QderivU[1]*(uPrev[delay_a+1,1] -u_Ol[1,1])^2 + 
                                              QderivU[2]*(uPrev[delay_df+1,2]-u_Ol[1,2])^2 )
            @NLexpression(mdl, costZ, 0.5*sum{Q[j]*sum{(z_Ol[i,j]-z_Ref[i,j])^2,i=2:N+1},j=1:4})
            @NLobjective(mdl, Min, costZ + derivCost)

            m.mdl=mdl
            m.z0=z0; m.z_Ref=z_Ref; m.c=c;
            m.z_Ol=z_Ol; m.u_Ol = u_Ol
            m.uPrev=uPrev
            return m
        end
    end

    type MdlKin

        mdl::JuMP.Model

        z0::Array{JuMP.NonlinearParameter,1}
        c::Array{JuMP.NonlinearParameter,1}
        selStates::Array{JuMP.NonlinearParameter,2}
        stateCost::Array{JuMP.NonlinearParameter,1}
        uPrev::Array{JuMP.NonlinearParameter,2}

        eps_lane::Array{JuMP.Variable,1}
        alpha::Array{JuMP.Variable,1}
        z_Ol::Array{JuMP.Variable,2}
        u_Ol::Array{JuMP.Variable,2}

        derivCost::JuMP.NonlinearExpression
        laneCost::JuMP.NonlinearExpression
        terminalCost::JuMP.NonlinearExpression
        slackCost::JuMP.NonlinearExpression
        a_his::Array{JuMP.NonlinearParameter,1}
        df_his::Array{JuMP.NonlinearParameter,1}

        GP_vy_e::Array{JuMP.NonlinearParameter,1}
        GP_psiDot_e::Array{JuMP.NonlinearParameter,1}

        function MdlKin(agent::Agent)
            m = new()
            dt   = get_param("controller/dt")
            L_a  = get_param("L_a")
            L_b  = get_param("L_b")
            u_lb = [-2    -18/180*pi]
            u_ub = [ 2     18/180*pi]
            z_lb = [-Inf -Inf -Inf -0.5] # 1.s 2.ey 3.epsi 4.v
            z_ub = [ Inf  Inf  Inf  6.0] # 1.s 2.ey 3.epsi 4.v
            
            c_f  = get_param("controller/c_f")
            ey_max = get_param("ey")*get_param("ey_tighten")/2

            N          = agent.mpcParams.N
            QderivZ    = agent.mpcParams.QderivZ
            QderivU    = agent.mpcParams.QderivU
            R          = agent.mpcParams.R
            Q_term_cost= agent.mpcParams.Q_term_cost
            Q_lane     = agent.mpcParams.Q_lane
            Q_slack    = agent.mpcParams.Q_slack

            Np = agent.SS.Np
            Nl = agent.SS.Nl

            mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.09,linear_solver="ma27"))

            @variable( mdl, z_Ol[1:(N+1),1:4])
            @variable( mdl, u_Ol[1:N,1:2])
            @variable( mdl, eps_lane[1:N+1] >= 0)   # eps for soft lane constraints
            @variable( mdl, alpha[1:Nl*Np] >= 0)    # coefficients of the convex hull

            for i=1:2
                for j=1:N
                    setlowerbound(u_Ol[j,i], u_lb[i])
                    setupperbound(u_Ol[j,i], u_ub[i])
                end
            end
            for j=1:N+1
                setlowerbound(z_Ol[j,4], z_lb[4])
                setupperbound(z_Ol[j,4], z_ub[4])
            end

            @NLparameter(mdl, z0[i=1:4] == 0)
            @NLparameter(mdl, c[1:N]==0)
            @NLparameter(mdl, uPrev[1:N,1:2] == 0)
            @NLparameter(mdl, selStates[1:Nl*Np,1:4] == 0)
            @NLparameter(mdl, statesCost[1:Nl*Np] == 0)   

            @NLconstraint(mdl, [i=2:N+1], z_Ol[i,2] <= ey_max + eps_lane[i])
            @NLconstraint(mdl, [i=2:N+1], z_Ol[i,2] >= -ey_max - eps_lane[i])
            @NLconstraint(mdl, sum{alpha[i], i=1:Nl*Np} == 1)
            delay_a = get_param("controller/delay_a")
            if agent.mpcParams.delay_a > 0
                @NLparameter(mdl, a_his[1:agent.mpcParams.delay_a] == 0)
                @NLconstraint(mdl, [i=1:agent.mpcParams.delay_a], u_Ol[i,1] == a_his[i])
                m.a_his  = a_his
            end
            delay_df = get_param("controller/delay_df")
            if agent.mpcParams.delay_df > 0
                @NLparameter(mdl, df_his[1:agent.mpcParams.delay_df] == 0)
                @NLconstraint(mdl, [i=1:agent.mpcParams.delay_df], u_Ol[i,2] == df_his[i])
                m.df_his = df_his
            end

            @NLparameter(mdl, GP_vy_e[i=1:N] == 0)
            @NLparameter(mdl, GP_psiDot_e[i=1:N] == 0)

            # @NLexpression(mdl, bta[i=1:N], atan( L_b / (L_a + L_b) * tan(u_Ol[i,2])))
            @NLexpression(mdl, bta[i=1:N],  L_a/(L_a + L_b)*u_Ol[i,2])
            @NLexpression(mdl, dsdt[i=1:N], z_Ol[i,4]*cos(z_Ol[i,3]+bta[i])/(1-z_Ol[i,2]*c[i]))

            @NLconstraint(mdl, [i=1:4], z_Ol[1,i] == z0[i])
            for i=1:N
                @NLconstraint(mdl, z_Ol[i+1,1] == z_Ol[i,1] + dt*dsdt[i])                                       # s
                @NLconstraint(mdl, z_Ol[i+1,2] == z_Ol[i,2] + dt*z_Ol[i,4]*sin(z_Ol[i,3]+bta[i])  )             # ey
                @NLconstraint(mdl, z_Ol[i+1,3] == z_Ol[i,3] + dt*(z_Ol[i,4]/L_b*sin(bta[i])-dsdt[i]*c[i])  )    # epsi
                @NLconstraint(mdl, z_Ol[i+1,4] == z_Ol[i,4] + dt*(u_Ol[i,1] - c_f*z_Ol[i,4]))                   # v
            end

            # for i=1:4
            #     @NLconstraint(mdl, z_Ol[N+1,i] == sum(alpha[j]*selStates[j,i] for j=1:Nl*Np))
            # end

            @NLexpression(mdl, derivCost, sum{QderivZ[j]*sum{(z_Ol[i,j]-z_Ol[i+1,j])^2,i=1:N},j=1:4} +
                                              QderivU[1]*sum{(u_Ol[i,1]-u_Ol[i+1,1])^2,i=delay_a+1:N-1} + 
                                              QderivU[2]*sum{(u_Ol[i,2]-u_Ol[i+1,2])^2,i=delay_df+1:N-1} + 
                                              QderivU[1]*(uPrev[delay_a+1,1] -u_Ol[delay_a+1, 1])^2 + 
                                              QderivU[2]*(uPrev[delay_df+1,2]-u_Ol[delay_df+1,2])^2 )
            @NLexpression(mdl, laneCost,     Q_lane*sum{1.0*eps_lane[i]+20.0*eps_lane[i]^2, i=2:N+1})
            @NLexpression(mdl, terminalCost, Q_term_cost*sum{alpha[i]*statesCost[i], i=1:Nl*Np})
            @NLexpression(mdl, slackCost,    sum{Q_slack[i]*(z_Ol[N+1,i]-sum{alpha[j]*selStates[j,i],j=1:Nl*Np})^2, i=1:4})

            @NLobjective(mdl, Min, derivCost+laneCost+terminalCost+slackCost)

            m.mdl = mdl
            m.z0 = z0
            m.c = c
            m.z_Ol = z_Ol
            m.u_Ol = u_Ol
            m.uPrev = uPrev

            m.derivCost   = derivCost
            m.laneCost    = laneCost
            m.terminalCost= terminalCost
            m.slackCost   = slackCost
            m.selStates   = selStates   
            m.stateCost   = statesCost  
            m.alpha       = alpha       
            m.GP_vy_e     = GP_vy_e
            m.GP_psiDot_e = GP_psiDot_e
            return m
        end
    end

    type MdlId

        mdl::JuMP.Model

        z0::Array{JuMP.NonlinearParameter,1}
        selStates::Array{JuMP.NonlinearParameter,2}
        stateCost::Array{JuMP.NonlinearParameter,1}
        c_Vx::Array{JuMP.NonlinearParameter,2}
        c_Vy::Array{JuMP.NonlinearParameter,2}
        c_Psi::Array{JuMP.NonlinearParameter,2}
        uPrev::Array{JuMP.NonlinearParameter,2}
        df_his::Array{JuMP.NonlinearParameter,1}
        a_his::Array{JuMP.NonlinearParameter,1}
        GP_vy_e::Array{JuMP.NonlinearParameter,1}
        GP_psiDot_e::Array{JuMP.NonlinearParameter,1}

        eps_lane::Array{JuMP.Variable,1}
        alpha::Array{JuMP.Variable,1}
        z_Ol::Array{JuMP.Variable,2}
        u_Ol::Array{JuMP.Variable,2}

        dsdt::Array{JuMP.NonlinearExpression,1}
        c::Array{JuMP.NonlinearParameter,1}

        derivCost::JuMP.NonlinearExpression
        laneCost::JuMP.NonlinearExpression
        terminalCost::JuMP.NonlinearExpression

        function MdlId(agent::Agent)
            m = new()
            dt      = get_param("controller/dt")
            L_a     = get_param("L_a")
            L_b     = get_param("L_b")
            ey_max  = get_param("ey")*get_param("ey_tighten")/2
            u_lb    = [ -2.0    -18/180*pi]
            u_ub    = [  2.0     18/180*pi]
            z_lb    = [-Inf -Inf -Inf    0 -Inf -Inf] # 1.s 2.ey 3.epsi 4.vx 5.vy 6.psi_dot
            z_ub    = [ Inf  Inf  Inf  6.0  Inf  Inf] # 1.s 2.ey 3.epsi 4.vx 5.vy 6.psi_dot

            N          = agent.mpcParams.N
            QderivZ    = agent.mpcParams.QderivZ
            QderivU    = agent.mpcParams.QderivU
            Q_term_cost= agent.mpcParams.Q_term_cost
            R          = agent.mpcParams.R
            Q          = agent.mpcParams.Q
            Q_lane     = agent.mpcParams.Q_lane
            Q_slack    = agent.mpcParams.Q_slack

            Np         = agent.SS.Np
            Nl         = agent.SS.Nl

            mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.09)) #,linear_solver="ma27"))#,check_derivatives_for_naninf="yes"))#,linear_solver="ma57",print_user_options="yes"))
            # 
            @variable( mdl, z_Ol[1:(N+1),1:6], start=0.1)
            @variable( mdl, u_Ol[1:N,1:2])
            @variable( mdl, eps_lane[1:N] >= 0)   # eps for soft lane constraints
            @variable( mdl, alpha[1:Nl*Np] >= 0)    # coefficients of the convex hull

            for j=1:N
                for i=1:2
                    setlowerbound(u_Ol[j,i], u_lb[i])
                    setupperbound(u_Ol[j,i], u_ub[i])
                end
                setlowerbound(z_Ol[j+1,4], z_lb[4])
                setupperbound(z_Ol[j+1,4], z_ub[4])
            end

            @NLparameter(mdl, z0[i=1:6] == 0)
            @NLparameter(mdl, GP_vy_e[i=1:N] == 0)
            @NLparameter(mdl, GP_psiDot_e[i=1:N] == 0)
            @NLparameter(mdl, c[1:N] == 0)
            @NLparameter(mdl, c_Vx[1:N,1:3]  == 0)
            @NLparameter(mdl, c_Vy[1:N,1:4]  == 0)
            @NLparameter(mdl, c_Psi[1:N,1:3] == 0)
            @NLparameter(mdl, uPrev[1:N,1:2] == 0)
            delay_a = agent.mpcParams.delay_a
            if delay_a > 0
                @NLparameter(mdl, a_his[1:agent.mpcParams.delay_a] == 0)
                @NLconstraint(mdl, [i=1:agent.mpcParams.delay_a], u_Ol[i,1] == a_his[i])
                m.a_his  = a_his
            end
            delay_df = agent.mpcParams.delay_df
            if delay_df > 0
                @NLparameter(mdl, df_his[1:agent.mpcParams.delay_df] == 0)
                @NLconstraint(mdl, [i=1:agent.mpcParams.delay_df], u_Ol[i,2] == df_his[i])
                m.df_his = df_his
            end

            @NLparameter(mdl, selStates[1:Nl*Np,1:6] == 0)
            @NLparameter(mdl, stateCost[1:Nl*Np] == 0)   

            @NLconstraint(mdl, [i=1:6], z_Ol[1,i] == z0[i])
            @NLconstraint(mdl, [i=2:N+1], z_Ol[i,2] <= ey_max + eps_lane[i-1])
            @NLconstraint(mdl, [i=2:N+1], z_Ol[i,2] >= -ey_max - eps_lane[i-1])
            @NLconstraint(mdl, sum{alpha[i],i=1:Nl*Np} == 1)

            @NLexpression(mdl, dsdt[i = 1:N], (z_Ol[i,4]*cos(z_Ol[i,3]) - z_Ol[i,5]*sin(z_Ol[i,3]))/(1-z_Ol[i,2]*c[i]))

            # for i=1:n_state
            #     @NLconstraint(mdl, z_Ol[N+1,i] == sum(alpha[j]*selStates[j,i] for j=1:Nl*Np))
            # end

            # System dynamics
            for i=1:N
                @NLconstraint(mdl, z_Ol[i+1,1]  == z_Ol[i,1] + dt * dsdt[i])                    # s
                @NLconstraint(mdl, z_Ol[i+1,2]  == z_Ol[i,2] + dt * (z_Ol[i,4]*sin(z_Ol[i,3]) + z_Ol[i,5]*cos(z_Ol[i,3])))  # eY
                @NLconstraint(mdl, z_Ol[i+1,3]  == z_Ol[i,3] + dt * (z_Ol[i,6]-dsdt[i]*c[i]))   # ePsi
                @NLconstraint(mdl, z_Ol[i+1,4]  == z_Ol[i,4] + c_Vx[i,1]*z_Ol[i,5]*z_Ol[i,6] + 
                                                               c_Vx[i,2]*z_Ol[i,4] +
                                                               c_Vx[i,3]*u_Ol[i,1])
                @NLconstraint(mdl, z_Ol[i+1,5]  == z_Ol[i,5] + c_Vy[i,1]*z_Ol[i,5]/z_Ol[i,4] + 
                                                               c_Vy[i,2]*z_Ol[i,4]*z_Ol[i,6] + 
                                                               c_Vy[i,3]*z_Ol[i,6]/z_Ol[i,4] + 
                                                               c_Vy[i,4]*u_Ol[i,2] + 
                                                               GP_vy_e[i])                      # vy
                @NLconstraint(mdl, z_Ol[i+1,6]  == z_Ol[i,6] + c_Psi[i,1]*z_Ol[i,6]/z_Ol[i,4] + 
                                                               c_Psi[i,2]*z_Ol[i,5]/z_Ol[i,4] + 
                                                               c_Psi[i,3]*u_Ol[i,2] + 
                                                               GP_psiDot_e[i])                  # psiDot
            end

            @NLexpression(mdl, derivCost, sum{QderivZ[j]*sum{(z_Ol[i,j]-z_Ol[i+1,j])^2,i=1:N},j=1:6} +
                                              QderivU[1]*sum{(u_Ol[i,1]-u_Ol[i+1,1])^2,i=delay_a+1:N-1} + 
                                              QderivU[2]*sum{(u_Ol[i,2]-u_Ol[i+1,2])^2,i=delay_df+1:N-1} + 
                                              QderivU[1]*(uPrev[delay_a+1,1] -u_Ol[delay_a+1, 1])^2 + 
                                              QderivU[2]*(uPrev[delay_df+1,2]-u_Ol[delay_df+1,2])^2 )
            @NLexpression(mdl, laneCost,    Q_lane*sum{1.0*eps_lane[i]+20.0*eps_lane[i]^2 , i=1:N})
            @NLexpression(mdl, terminalCost,Q_term_cost*sum{alpha[i]*stateCost[i] , i=1:Nl*Np})
            @NLexpression(mdl, slackCost,   sum{Q_slack[i]*(z_Ol[N+1,i]-sum{alpha[j]*selStates[j,i],j=1:Nl*Np})^2, i=1:6})

            @NLobjective(mdl, Min, derivCost + laneCost +  terminalCost + slackCost)

            m.mdl = mdl
            m.z0 = z0
            m.c = c
            m.z_Ol = z_Ol
            m.u_Ol = u_Ol
            m.c_Vx = c_Vx
            m.c_Vy = c_Vy
            m.c_Psi = c_Psi
            m.uPrev = uPrev

            m.derivCost = derivCost
            m.laneCost = laneCost
            m.terminalCost= terminalCost 
            m.selStates   = selStates    
            m.stateCost   = stateCost   
            m.GP_vy_e     = GP_vy_e
            m.GP_psiDot_e = GP_psiDot_e
            m.alpha       = alpha
            return m
        end
    end

    type MdlKinLin

        mdl::JuMP.Model

        selStates::Array{JuMP.NonlinearParameter,2}
        statesCost::Array{JuMP.NonlinearParameter,1}
        uPrev::Array{JuMP.NonlinearParameter,2}
        zPrev::Array{JuMP.NonlinearParameter,2}
        df_his::Array{JuMP.NonlinearParameter,1}

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

        function MdlKinLin(agent::Agent)
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
            z_lb = [-Inf -Inf -pi -0.5] # 1.s 2.ey 3.epsi 4.v
            z_ub = [ Inf  Inf  pi  2.5] # 1.s 2.ey 3.epsi 4.v

            c_f        = modelParams.c_f

            ey_max  = get_param("ey")*get_param("ey_tighten")/2

            N          = mpcParams.N                           # Prediction horizon
            QderivZ    = mpcParams.QderivZ::Array{Float64,1}   # weights for the derivative cost on the states
            QderivU    = mpcParams.QderivU::Array{Float64,1}   # weights for the derivative cost on the control inputs
            R          = mpcParams.R::Array{Float64,1}         # weights on the control inputs
            Q          = mpcParams.Q::Array{Float64,1}         # weights on the states for path following
            Q_term_cost= mpcParams.Q_term_cost::Float64        # weights on the terminal states
            Q_lane     = mpcParams.Q_lane::Float64             # weight on the soft constraint on the lane
            Q_vel      = mpcParams.Q_vel::Float64             # weight on the soft constraint for the max velocity
            Q_slack    = mpcParams.Q_slack

            Np         = selectedStates.Np              # how many states to select
            Nl         = selectedStates.Nl               # how many previous laps to select

            mdl = Model(solver = IpoptSolver(print_level=0,linear_solver="ma27",max_cpu_time=0.09))#,check_derivatives_for_naninf="yes"))#,linear_solver="ma57",print_user_options="yes"))

            @NLparameter( mdl, zPrev[1:N+1,1:n_state] == 0)
            @NLparameter( mdl, uPrev[1:N,1:n_input] == 0)
            @NLparameter( mdl, z_linear[1:N+1,1:n_state] == 0) # reference variables, which need to be added back to delta decision variables
            @NLparameter( mdl, u_linear[1:N,1:n_input] == 0)
            @NLparameter( mdl, A[1:n_state,1:n_state,1:N]==0)
            @NLparameter( mdl, B[1:n_state,1:n_input,1:N]==0)
            @NLparameter( mdl, selStates[1:Nl*Np,1:n_state]==0)
            @NLparameter( mdl, statesCost[1:Nl*Np]==0)
            @NLparameter( mdl, df_his[1:mpcParams.delay_df] == 0)

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
            
            @NLconstraint(mdl, u_Ol[1,1]+u_linear[1,1] == uPrev[2,1]) # acceleration delay is 1
            @NLconstraint(mdl, [i=1:mpcParams.delay_df], u_Ol[i,2]+u_linear[i,2] == df_his[i]) # steering delay is 1 for simulation and 3 for experiment

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
            m.df_his = df_his

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

    type MdlDynLin

        mdl::JuMP.Model

        selStates::Array{JuMP.NonlinearParameter,2}
        stateCost::Array{JuMP.NonlinearParameter,1}
        uPrev::Array{JuMP.NonlinearParameter,2}

        A::Array{JuMP.NonlinearParameter,3}
        B::Array{JuMP.NonlinearParameter,3}

        eps_lane::Array{JuMP.Variable,1}
        alpha::Array{JuMP.Variable,1}
        z_Ol::Array{JuMP.Variable,2}
        u_Ol::Array{JuMP.Variable,2}
        z_linear::Array{JuMP.NonlinearParameter,2}
        u_linear::Array{JuMP.NonlinearParameter,2}

        df_his::Array{JuMP.NonlinearParameter,1}
        a_his::Array{JuMP.NonlinearParameter,1}

        derivCost
        laneCost
        terminalCost

        function MdlDynLin(agent::Agent)
            m = new()
            dt      = get_param("controller/dt")
            L_a     = get_param("L_a")
            L_b     = get_param("L_b")
            ey_max  = get_param("ey")*get_param("ey_tighten")/2
            u_lb    = [ -2.0    -18/180*pi]
            u_ub    = [  2.0     18/180*pi]
            z_lb    = [-Inf -Inf -Inf    0 -Inf -Inf] # 1.s 2.ey 3.epsi 4.vx 5.vy 6.psi_dot
            z_ub    = [ Inf  Inf  Inf  3.0  Inf  Inf] # 1.s 2.ey 3.epsi 4.vx 5.vy 6.psi_dot
            
            c_f     = get_param("controller/c_f")
            ey_max  = get_param("ey")*get_param("ey_tighten")/2

            N          = agent.mpcParams.N
            QderivZ    = agent.mpcParams.QderivZ
            QderivU    = agent.mpcParams.QderivU
            R          = agent.mpcParams.R
            Q          = agent.mpcParams.Q
            Q_term_cost= agent.mpcParams.Q_term_cost
            Q_lane     = agent.mpcParams.Q_lane
            Q_slack    = agent.mpcParams.Q_slack

            Np         = agent.SS.Np::Int64
            Nl         = agent.SS.Nl::Int64

            mdl = Model(solver = IpoptSolver(print_level=0,linear_solver="ma27",max_cpu_time=0.09))

            @NLparameter( mdl, uPrev[1:N,1:2]       == 0)
            @NLparameter( mdl, z_linear[1:N+1,1:6]  == 0)
            @NLparameter( mdl, u_linear[1:N,1:2]    == 0)
            @NLparameter( mdl, A[1:6,1:6,1:N]       == 0)
            @NLparameter( mdl, B[1:6,1:2,1:N]       == 0)
            @NLparameter( mdl, selStates[1:Nl*Np,1:6]==0)
            @NLparameter( mdl, stateCost[1:Nl*Np]   == 0)
            
            @variable( mdl, z_Ol[1:N,1:6])
            @variable( mdl, u_Ol[1:N,1:2])
            @variable( mdl, eps_lane[1:N] >= 0)
            @variable( mdl, alpha[1:Nl*Np]>= 0)

            delay_a = agent.mpcParams.delay_a
            # if delay_a > 0
            #     @NLparameter(mdl, a_his[1:delay_a] == 0)
            #     @NLconstraint(mdl, [i=1:delay_a], u_Ol[i,1] + u_linear[i,1] == a_his[i])
            #     m.a_his  = a_his
            # end
            delay_df = agent.mpcParams.delay_df
            # if delay_df > 0
            @NLparameter(mdl, df_his[1:delay_df] == 0)
            # @NLconstraint(mdl, [i=1:delay_df], u_Ol[i,2] + u_linear[i,2] == df_his[i])
            m.df_his = df_his
            # end

            @NLconstraint(mdl, u_Ol[1,1]+u_linear[1,1] == uPrev[2,1]) # acceleration delay is 1
            @NLconstraint(mdl, [i=1:delay_df], u_Ol[i,2]+u_linear[i,2] == df_his[i]) # steering delay is 1 for simulation and 3 for experiment


            @NLconstraint(mdl, [i=1:N], z_Ol[i,2] + z_linear[i+1,2] <= ey_max + eps_lane[i])
            @NLconstraint(mdl, [i=1:N], z_Ol[i,2] + z_linear[i+1,2] >= -ey_max - eps_lane[i])
            @constraint(mdl, sum{alpha[i] , i=1:Nl*Np} == 1)

            @NLconstraint(mdl, [i=1:N], z_Ol[i,4] + z_linear[i+1,4] <= z_ub[4])
            @NLconstraint(mdl, [i=1:N], z_Ol[i,4] + z_linear[i+1,4] >= z_lb[4])
            for j=1:2
                @NLconstraint(mdl, [i=1:N], u_Ol[i,j] + u_linear[i,j] <= u_ub[j])
                @NLconstraint(mdl, [i=1:N], u_Ol[i,j] + u_linear[i,j] >= u_lb[j])
            end

            for i=1:N-1
                @NLconstraint(mdl, u_Ol[i+1,2]+u_linear[i+1,2]-(u_Ol[i,2]+u_linear[i,2]) <= 0.12)
                @NLconstraint(mdl, u_Ol[i+1,2]+u_linear[i+1,2]-(u_Ol[i,2]+u_linear[i,2]) >= -0.12)
            end

            # System dynamics
            for i=1:N-1
                for j=1:6
                    @NLconstraint(mdl, z_Ol[i+1,j] == sum{A[j,k,i+1]*z_Ol[i,k],   k=1:6}+
                                                      sum{B[j,k,i+1]*u_Ol[i+1,k], k=1:2})
                end
            end
            for j=1:6
                @NLconstraint(mdl, z_Ol[1,j] == sum{B[j,k,1]*u_Ol[1,k], k=1:2} )
            end
            
            # for i=1:n_state
            #     @NLconstraint(mdl, z_Ol[N,i] + z_linear[N+1,i] == sum{alpha[j]*selStates[j,i] , j=1:Nl*Np})
            # end
            @NLexpression(mdl, derivCost, sum{QderivZ[j]*(sum{(z_Ol[i,j]+z_linear[i+1,j]-z_Ol[i+1,j]-z_linear[i+2,j])^2,i=1:N-1}+
                                                          (z_Ol[1,j]+z_linear[2,j]-z_linear[1,j])^2), j=1:6} +
                                              QderivU[1]*sum{(u_Ol[i,1]+u_linear[i,1]-u_Ol[i+1,1]-u_linear[i+1,1])^2,i=delay_a+1:N-1} + 
                                              QderivU[2]*sum{(u_Ol[i,2]+u_linear[i,1]-u_Ol[i+1,2]-u_linear[i+1,1])^2,i=delay_df+1:N-1})# + 
                                              # QderivU[1]*(uPrev[delay_a+1,1] -u_Ol[delay_a+1, 1]-u_linear[delay_a+1, 1])^2 + 
                                              # QderivU[2]*(uPrev[delay_df+1,2]-u_Ol[delay_df+1,2]-u_linear[delay_df+1,1])^2 )

            @NLexpression(mdl, laneCost,    Q_lane*sum{10.0*eps_lane[i]+50.0*eps_lane[i]^2, i=1:N})
            @NLexpression(mdl, terminalCost,Q_term_cost*sum{alpha[i]*stateCost[i] , i=1:Nl*Np})
            @NLexpression(mdl, slackCost,   sum{Q_slack[i]*(z_Ol[N,i]+z_linear[N+1,i]-sum{alpha[j]*selStates[j,i],j=1:Nl*Np})^2, i=1:6})

            @NLobjective(mdl, Min, derivCost + laneCost +  terminalCost + slackCost)
            
            m.mdl = mdl
            m.z_Ol = z_Ol
            m.u_Ol = u_Ol
            m.z_linear = z_linear
            m.u_linear = u_linear
            m.uPrev = uPrev

            m.laneCost = laneCost
            m.terminalCost= terminalCost
            m.selStates   = selStates   
            m.stateCost   = stateCost   
            m.alpha       = alpha       
            m.eps_lane    = eps_lane

            m.A=A
            m.B=B

            return m
        end
    end
end # end of module, mpcModels

module solveMpcProblem
using JuMP
using Types
using ControllerHelper
using mpcModels
import CarSim:car_pre_dyn
    function solvePf(mdl::MdlPf,agent::Agent)
        s=vcat(agent.posInfo.s,agent.mpcSol.z_prev[3:end,1])
        curvature=curvature_prediction(s,agent.track)
        z_curr = [agent.posInfo.s,agent.posInfo.ey,agent.posInfo.epsi,agent.posInfo.v]

        if agent.mpcParams.delay_df > 0
            setvalue(mdl.df_his,agent.mpcSol.df_his)
        end
        if agent.mpcParams.delay_a > 0
            setvalue(mdl.a_his, agent.mpcSol.a_his)
        end

        setvalue(mdl.z0,z_curr)
        setvalue(mdl.c,curvature)
        setvalue(mdl.uPrev, agent.mpcSol.u_prev)

        agent.mpcSol.sol_status = solve(mdl.mdl)
        agent.mpcSol.u          = getvalue(mdl.u_Ol)
        agent.mpcSol.z[:,1:4]   = getvalue(mdl.z_Ol)
        agent.mpcSol.a_x = agent.mpcSol.u[1+agent.mpcParams.delay_a,1]
        agent.mpcSol.d_f = agent.mpcSol.u[1+agent.mpcParams.delay_df,2]
        agent.cmd.motor  = agent.mpcSol.a_x
        agent.cmd.servo  = agent.mpcSol.d_f

        push!(agent.mpcSol.a_his,agent.mpcSol.a_x)
        shift!(agent.mpcSol.a_his)
        push!(agent.mpcSol.df_his,agent.mpcSol.d_f)
        shift!(agent.mpcSol.df_his)
    end

    function solveFd(mdl::MdlPf,agent::Agent,v_ref::Float64)

        z_final=car_sim_kin(z_prev[end,:],u_prev[end,:],track,modelParams)
        z_curvature=vcat(z_curr',z_prev[3:end,:],z_final)

        curvature=curvature_prediction(z_curvature,track)
        z_ref = hcat(zeros(mpcParams_pF.N+1,3),v_ref*ones(mpcParams_pF.N+1,1))

        # Update current initial condition, curvature and previous input
        setvalue(mdl.z0,z_curr)
        setvalue(mdl.c,curvature)
        setvalue(mdl.df_his,mpcSol.df_his)
        setvalue(mdl.uPrev,u_prev)
        setvalue(mdl.z_Ref,z_ref)

        # Solve Problem and return solution
        sol_status  = solve(mdl.mdl)
        sol_u       = getvalue(mdl.u_Ol)
        sol_z       = getvalue(mdl.z_Ol)
        return sol_z,sol_u,sol_status
    end

    function solveKin(mdl::MdlKin,agent::Agent) 

        s=vcat(agent.posInfo.s,agent.mpcSol.z_prev[3:end,1])
        curvature=curvature_prediction(s,agent.track)
        z_curr = [agent.posInfo.s,agent.posInfo.ey,agent.posInfo.epsi,agent.posInfo.v]

        if agent.mpcParams.delay_df > 0
            setvalue(mdl.df_his,agent.mpcSol.df_his)
        end
        if agent.mpcParams.delay_a > 0
            setvalue(mdl.a_his, agent.mpcSol.a_his)
        end

        setvalue(mdl.z0,            z_curr)
        setvalue(mdl.uPrev,         agent.mpcSol.u_prev)
        setvalue(mdl.c,             curvature)
        setvalue(mdl.selStates,     agent.SS.selStates)
        setvalue(mdl.stateCost,     agent.SS.stateCost)
        setvalue(mdl.GP_vy_e,       agent.gpData.GP_vy_e)
        setvalue(mdl.GP_psiDot_e,   agent.gpData.GP_psiDot_e)

        agent.mpcSol.sol_status  = solve(mdl.mdl)
        agent.mpcSol.u = getvalue(mdl.u_Ol)
        agent.mpcSol.z = getvalue(mdl.z_Ol)

        agent.mpcSol.a_x = agent.mpcSol.u[1+agent.mpcParams.delay_a,1] 
        agent.mpcSol.d_f = agent.mpcSol.u[1+agent.mpcParams.delay_df,2]
        agent.cmd.motor = agent.mpcSol.a_x 
        agent.cmd.servo = agent.mpcSol.d_f 

        push!(agent.mpcSol.a_his,agent.mpcSol.a_x)
        shift!(agent.mpcSol.a_his)
        push!(agent.mpcSol.df_his,agent.mpcSol.d_f)
        shift!(agent.mpcSol.df_his)
    end

    function solveId(mdl::MdlId,agent::Agent)

        s=vcat(agent.posInfo.s,agent.mpcSol.z_prev[3:end,1])
        curvature=curvature_prediction(s,agent.track)
        z_curr = [agent.posInfo.s,agent.posInfo.ey,agent.posInfo.epsi,agent.posInfo.vx,agent.posInfo.vy,agent.posInfo.psiDot]

        if agent.mpcParams.delay_df > 0
            setvalue(mdl.df_his,agent.mpcSol.df_his)
        end
        if agent.mpcParams.delay_a > 0
            setvalue(mdl.a_his, agent.mpcSol.a_his)
        end

        setvalue(mdl.z0,            z_curr)
        setvalue(mdl.uPrev,         agent.mpcSol.u_prev)
        setvalue(mdl.c,             curvature)
        setvalue(mdl.selStates,     agent.SS.selStates)
        setvalue(mdl.stateCost,     agent.SS.stateCost)
        setvalue(mdl.GP_vy_e,       agent.gpData.GP_vy_e)
        setvalue(mdl.GP_psiDot_e,   agent.gpData.GP_psiDot_e)

        setvalue(mdl.c_Vx, agent.sysID.c_Vx)
        setvalue(mdl.c_Vy, agent.sysID.c_Vy)
        setvalue(mdl.c_Psi,agent.sysID.c_Psi)

        agent.mpcSol.sol_status  = solve(mdl.mdl)
        agent.mpcSol.u = getvalue(mdl.u_Ol)
        agent.mpcSol.z = getvalue(mdl.z_Ol)

        agent.mpcSol.a_x = agent.mpcSol.u[1+agent.mpcParams.delay_a,1] 
        agent.mpcSol.d_f = agent.mpcSol.u[1+agent.mpcParams.delay_df,2]
        agent.cmd.motor = agent.mpcSol.a_x 
        agent.cmd.servo = agent.mpcSol.d_f 

        push!(agent.mpcSol.a_his,agent.mpcSol.a_x)
        shift!(agent.mpcSol.a_his)
        push!(agent.mpcSol.df_his,agent.mpcSol.d_f)
        shift!(agent.mpcSol.df_his)
    end

    function solveKinLin(mdl::MdlKinLin,agent::Agent)

        # Interface: from absolute value to relative state value
        n_state=4; n_input=2;

        selStates       = selectedStates.selStates
        statesCost      = selectedStates.statesCost

        # for i=1:mpcParams.N
        #     for j=1:n_state
        #         JuMP.fix(mdl.z_linear[i,j],z_linear[i,j])
        #     end
        #     for j=1:n_input
        #         JuMP.fix(mdl.u_linear[i,j],u_linear[i,j])
        #     end
        # end
        # for j=1:n_state
        #     JuMP.fix(mdl.z_linear[mpcParams.N+1,j],z_linear[mpcParams.N+1,j])
        # end
        setvalue(mdl.z_linear,z_linear)
        setvalue(mdl.u_linear,u_linear)
        setvalue(mdl.df_his,mpcSol.df_his)


        # uPrev=u_prev; zPrev=z_prev
        # i=1; j=1; k=1;
        # mdl=mdl_convhull

        A=zeros(n_state,n_state,mpcParams.N)
        B=zeros(n_state,n_input,mpcParams.N)
        L_a=modelParams.l_A; L_b=modelParams.l_B
        dt=modelParams.dt; c_f=modelParams.c_f
        curvature=curvature_prediction(z_linear,track)
        for k=1:mpcParams.N
            bta_val=atan((L_a*tan(u_linear[k,2]))/(L_a + L_b))

            # A[1,1,k]= dt*z_linear[k,2]*z_linear[k,4]*cos(z_linear[k,3] + bta_val)*curvature_deri/(z_linear[k,2]*(curvature) - 1)^2 + 1
            A[1,1,k]= 1
            A[1,2,k]= dt*z_linear[k,4]*cos(z_linear[k,3] + bta_val)*curvature[k]/(z_linear[k,2]*(curvature[k]) - 1)^2
            A[1,3,k]= dt*z_linear[k,4]*sin(z_linear[k,3] + bta_val)/(z_linear[k,2]*(curvature[k]) - 1)
            A[1,4,k]=-dt*cos(z_linear[k,3] + bta_val)/(z_linear[k,2]*(curvature[k]) - 1)
            A[2,1,k]= 0
            A[2,2,k]= 1
            A[2,3,k]= dt*z_linear[k,4]*cos(z_linear[k,3] + bta_val)
            A[2,4,k]= dt*sin(z_linear[k,3] + bta_val)
            # A[3,1,k]= dt*( (z_linear[k,4]*cos(z_linear[k,3] + bta_val)*curvature_deri)/(z_linear[k,2]*curvature - 1)-
                           # (z_linear[k,2]*z_linear[k,4]*cos(z_linear[k,3] + bta_val)*curvature_deri*curvature)/(z_linear[k,2]*curvature - 1)^2 )
            A[3,1,k]= 0
            A[3,2,k]=-(dt*z_linear[k,4]*cos(z_linear[k,3] + bta_val)*(curvature[k])^2)/(z_linear[k,2]*(curvature[k]) - 1)^2
            A[3,3,k]=-(dt*z_linear[k,4]*sin(z_linear[k,3] + bta_val)*(curvature[k]))/(z_linear[k,2]*(curvature[k]) - 1) + 1
            A[3,4,k]=dt*((cos(z_linear[k,3] + bta_val)*(curvature[k]))/(z_linear[k,2]*(curvature[k]) - 1) + sin(bta_val)/L_b)
            A[4,1,k]= 0
            A[4,2,k]= 0
            A[4,3,k]= 0
            A[4,4,k]= 1-c_f*dt
            B[1,1,k]=0
            B[2,1,k]=0
            B[3,1,k]=0
            B[4,1,k]=dt
            B[1,2,k]=(L_a*dt*z_linear[k,4]*sin(z_linear[k,3] + atan((L_a*tan(u_linear[k,2]))/(L_a + L_b)))*(tan(u_linear[k,2])^2 + 1))/(((L_a^2*tan(u_linear[k,2])^2)/(L_a + L_b)^2 + 1)*(z_linear[k,2]*curvature[k] - 1)*(L_a + L_b))
            B[2,2,k]=(L_a*dt*z_linear[k,4]*cos(z_linear[k,3] + atan((L_a*tan(u_linear[k,2]))/(L_a + L_b)))*(tan(u_linear[k,2])^2 + 1))/(((L_a^2*tan(u_linear[k,2])^2)/(L_a + L_b)^2 + 1)*(L_a + L_b))
            B[3,2,k]=-dt*((L_a^3*z_linear[k,4]*tan(u_linear[k,2])^2*(tan(u_linear[k,2])^2 + 1))/(L_b*((L_a^2*tan(u_linear[k,2])^2)/(L_a + L_b)^2 + 1)^(3/2)*(L_a + L_b)^3) - (L_a*z_linear[k,4]*(tan(u_linear[k,2])^2 + 1))/(L_b*((L_a^2*tan(u_linear[k,2])^2)/(L_a + L_b)^2 + 1)^(1/2)*(L_a + L_b)) + (L_a*z_linear[k,4]*sin(z_linear[k,3] + atan((L_a*tan(u_linear[k,2]))/(L_a + L_b)))*(tan(u_linear[k,2])^2 + 1)*curvature[k])/(((L_a^2*tan(u_linear[k,2])^2)/(L_a + L_b)^2 + 1)*(z_linear[k,2]*curvature[k] - 1)*(L_a + L_b)))
            B[4,2,k]=0

            setvalue(mdl.A,A)
            setvalue(mdl.B,B)
            # for i=1:n_state
            #     for j=1:n_state
            # #         A[i,j,k] = subs(mdl.A_s[i,j],mdl.z_s[1]=>z_linear[1],mdl.z_s[2]=>z_linear[k,2],
            # #                                      mdl.z_s[3]=>z_linear[k,3],mdl.z_s[4]=>z_linear[k,4],
            # #                                      mdl.u_s[1]=>u_linear[k,1],mdl.u_s[2]=>u_linear[k,2],
            # #                                      mdl.coeff_s[1]=>curvatureCoeff[1],mdl.coeff_s[2]=>curvatureCoeff[2],
            # #                                      mdl.coeff_s[3]=>curvatureCoeff[3],mdl.coeff_s[4]=>curvatureCoeff[4],
            # #                                      mdl.coeff_s[5]=>curvatureCoeff[5],mdl.coeff_s[6]=>curvatureCoeff[6],
            # #                                      mdl.coeff_s[7]=>curvatureCoeff[7])
            #         JuMP.fix(mdl.A[i,j,k],A[i,j,k])
            #     end
            #     for j=1:n_input
            #         # B[i,j,k] = subs(mdl.B_s[i,j],mdl.z_s[1]=>z_linear[k,1],mdl.z_s[2]=>z_linear[k,2],
            #         #                              mdl.z_s[3]=>z_linear[k,3],mdl.z_s[4]=>z_linear[k,4],
            #         #                              mdl.u_s[1]=>u_linear[k,1],mdl.u_s[2]=>u_linear[k,2],
            #         #                              mdl.coeff_s[1]=>curvatureCoeff[1],mdl.coeff_s[2]=>curvatureCoeff[2],
            #         #                              mdl.coeff_s[3]=>curvatureCoeff[3],mdl.coeff_s[4]=>curvatureCoeff[4],
            #         #                              mdl.coeff_s[5]=>curvatureCoeff[5],mdl.coeff_s[6]=>curvatureCoeff[6],
            #         #                              mdl.coeff_s[7]=>curvatureCoeff[7])
            #         JuMP.fix(mdl.B[i,j,k],B[i,j,k])
            #     end
            # end
        end

        # for i=1:selectedStates.Nl*selectedStates.Np
        # for i=1:size(selectedStates.selStates,1)
        #     for j=1:n_state
        #         JuMP.fix(mdl.selStates[i,j],selStates[i,j])
        #     end
        #     JuMP.fix(mdl.statesCost[i],statesCost[i])
        # end
        # for i=1:n_state
        #     JuMP.fix(mdl.zPrev[i],zPrev[1,i])
        # end
        # JuMP.fix(mdl.uPrev[1],uPrev[1,1])
        # JuMP.fix(mdl.uPrev[2],uPrev[1,2])
        setvalue(mdl.uPrev,uPrev)
        setvalue(mdl.selStates,selStates)
        setvalue(mdl.statesCost,statesCost)


        # Solve Problem and return solution
        sol_status  = solve(mdl.mdl)
        sol_u       = getvalue(mdl.u_Ol) + u_linear
        sol_z       = vcat(z_linear[1,:], getvalue(mdl.z_Ol) + z_linear[2:end,:])
        return sol_z,sol_u,sol_status
    end

    function solveDynLin(mdl::MdlDynLin,agent::Agent)
        z_curr = [agent.posInfo.s,agent.posInfo.ey,agent.posInfo.epsi,agent.posInfo.vx,agent.posInfo.vy,agent.posInfo.psiDot]
        z_linear,~,~ = car_pre_dyn(z_curr,agent.mpcSol.u,agent.track,agent.modelParams,6)
        # z_linear = vcat(agent.mpcSol.z_prev[2:end,:],agent.mpcSol.z_prev[end,:])
        u_linear = agent.mpcSol.u_prev
        # z_linear[1:end-1,5] += agent.gpData.GP_vy_e
        # z_linear[1:end-1,6] += agent.gpData.GP_psiDot_e
        # println(z_linear)
        # println(u_linear)
        println(round(agent.SS.selStates,4))
        curvature=curvature_prediction(z_linear[:,1],agent.track)
        println(curvature)
        setvalue(mdl.z_linear,z_linear)
        setvalue(mdl.u_linear,u_linear)

        L_a=agent.modelParams.L_a
        L_b=agent.modelParams.L_b
        dt =agent.modelParams.dt
        c_f=agent.modelParams.c_f
        m  =agent.modelParams.m
        I_z=agent.modelParams.I_z
        B  =agent.modelParams.B
        C  =agent.modelParams.C
        g  =agent.modelParams.g
        mu =agent.modelParams.mu
        
        m=1.98; I_z=0.03; 
        B = 6.0; C = 1.6; mu = 0.8; g = 9.81

        A_m=zeros(6,6,agent.mpcParams.N)
        B_m=zeros(6,2,agent.mpcParams.N)
        for k=1:agent.mpcParams.N
            A_m[1,1,k]= 1
            A_m[1,2,k]= (dt*(z_linear[k,4]*cos(z_linear[k,3]) - z_linear[k,5]*sin(z_linear[k,3]))*curvature[k])/(z_linear[k,2]*curvature[k] - 1)^2
            A_m[1,3,k]= (dt*(z_linear[k,5]*cos(z_linear[k,3]) + z_linear[k,4]*sin(z_linear[k,3])))/(z_linear[k,2]*curvature[k] - 1)
            A_m[1,4,k]= -(dt*cos(z_linear[k,3]))/(z_linear[k,2]*curvature[k] - 1)
            A_m[1,5,k]= (dt*sin(z_linear[k,3]))/(z_linear[k,2]*curvature[k] - 1)
            A_m[1,6,k]= 0
            A_m[2,1,k]= 0
            A_m[2,2,k]= 1
            A_m[2,3,k]= dt*(z_linear[k,4]*cos(z_linear[k,3])-z_linear[k,5]*sin(z_linear[k,3]))
            A_m[2,4,k]= dt*sin(z_linear[k,3])
            A_m[2,5,k]= dt*cos(z_linear[k,3])
            A_m[2,6,k]= 0
            # A_m[3,1,k]= dt*(((z_linear[k,4]*cos(z_linear[k,3]) - z_linear[k,5]*sin(z_linear[k,3]))*curvature_deri)/(z_linear[k,2]*curvature - 1) - (z_linear[k,2]*(z_linear[k,4]*cos(z_linear[k,3]) - z_linear[k,5]*sin(z_linear[k,3]))*curvature_deri*curvature)/(z_linear[k,2]*curvature - 1)^2)
            A_m[3,1,k]=0
            A_m[3,2,k]= -(dt*(z_linear[k,4]*cos(z_linear[k,3]) - z_linear[k,5]*sin(z_linear[k,3]))*curvature[k]^2)/(z_linear[k,2]*curvature[k] - 1)^2
            A_m[3,3,k]= 1 - (dt*(z_linear[k,5]*cos(z_linear[k,3]) + z_linear[k,4]*sin(z_linear[k,3]))*curvature[k])/(z_linear[k,2]*curvature[k] - 1)
            A_m[3,4,k]= (dt*cos(z_linear[k,3])*curvature[k])/(z_linear[k,2]*curvature[k] - 1)
            A_m[3,5,k]= -(dt*sin(z_linear[k,3])*curvature[k])/(z_linear[k,2]*curvature[k] - 1)
            A_m[3,6,k]= dt
            A_m[4,1,k]= 0
            A_m[4,2,k]= 0
            A_m[4,3,k]= 0
            A_m[4,4,k]= 1 - c_f*dt
            A_m[4,5,k]= dt*z_linear[k,6]
            A_m[4,6,k]= dt*z_linear[k,5]
            A_m[5,1,k]= 0
            A_m[5,2,k]= 0
            A_m[5,3,k]= 0
            A_m[5,4,k]= -dt*(z_linear[k,6] + ((B*C*(-g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])))*(z_linear[k,5] - L_a*z_linear[k,6]))/(2*z_linear[k,4]^2*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] -L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) + (B*C*(-g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2])*(z_linear[k,5] + L_a*z_linear[k,6]))/(2*z_linear[k,4]^2*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)))/m)

            A_m[5,5,k]= (dt*((B*C*(-g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4]))))/(2*z_linear[k,4]*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] - L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) + (B*C*(-g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*z_linear[k,4]*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1))))/m + 1

            A_m[5,6,k]= -dt*(z_linear[k,4] + ((B*C*L_a*(-g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4]))))/(2*z_linear[k,4]*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] - L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) - (B*C*L_a*(-g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*z_linear[k,4]*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)))/m)

            A_m[6,1,k]= 0
            A_m[6,2,k]= 0
            A_m[6,3,k]= 0
            A_m[6,4,k]= -(dt*((B*C*L_a*(g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])))*(z_linear[k,5] - L_a*z_linear[k,6]))/(2*z_linear[k,4]^2*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] - L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) - (B*C*L_a*(g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2])*(z_linear[k,5] + L_a*z_linear[k,6]))/(2*z_linear[k,4]^2*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1))))/I_z

            A_m[6,5,k]= (dt*((B*C*L_a*(g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4]))))/(2*z_linear[k,4]*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] - L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) - (B*C*L_a*(g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*z_linear[k,4]*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1))))/I_z


            A_m[6,6,k]= -(dt*((B*C*L_a^2*(g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4]))))/(2*z_linear[k,4]*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] - L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) + (B*C*L_a^2*(g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*z_linear[k,4]*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1))))/I_z +1

            B_m[1,1,k]= 0
            B_m[1,2,k]= 0
            B_m[2,1,k]= 0
            B_m[2,2,k]= 0
            B_m[3,1,k]= 0
            B_m[3,2,k]= 0
            B_m[4,1,k]= dt
            B_m[4,2,k]= 0
            B_m[5,1,k]= 0
            B_m[5,2,k]= -(dt*((g*m*mu*sin(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*sin(u_linear[k,2]))/2 - (B*C*g*m*mu*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1))))/m
            B_m[6,1,k]= 0
            B_m[6,2,k]= -(dt*((L_a*g*m*mu*sin(u_linear[k,2])*sin(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4])))))/2 - (B*C*L_a*g*m*mu*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1))))/I_z
        
            # A_m[1,1,k]= 1
            # A_m[1,2,k]= (dt*(z_linear[k,4]*cos(z_linear[k,3]) - z_linear[k,5]*sin(z_linear[k,3]))*curvature[k])/(z_linear[k,2]*curvature[k] - 1)^2
            # A_m[1,3,k]= (dt*(z_linear[k,5]*cos(z_linear[k,3]) + z_linear[k,4]*sin(z_linear[k,3])))/(z_linear[k,2]*curvature[k] - 1)
            # A_m[1,4,k]= -(dt*cos(z_linear[k,3]))/(z_linear[k,2]*curvature[k] - 1)
            # A_m[1,5,k]= (dt*sin(z_linear[k,3]))/(z_linear[k,2]*curvature[k] - 1)
            # A_m[1,6,k]= 0
            # A_m[2,1,k]= 0
            # A_m[2,2,k]= 1
            # A_m[2,3,k]= dt*(z_linear[k,4]*cos(z_linear[k,3])-z_linear[k,5]*sin(z_linear[k,3]))
            # A_m[2,4,k]= dt*sin(z_linear[k,3])
            # A_m[2,5,k]= dt*cos(z_linear[k,3])
            # A_m[2,6,k]= 0
            # A_m[3,1,k]= 0
            # A_m[3,2,k]= -(dt*(z_linear[k,4]*cos(z_linear[k,3]) - z_linear[k,5]*sin(z_linear[k,3]))*curvature[k]^2)/(z_linear[k,2]*curvature[k] - 1)^2
            # A_m[3,3,k]= 1 - (dt*(z_linear[k,5]*cos(z_linear[k,3]) + z_linear[k,4]*sin(z_linear[k,3]))*curvature[k])/(z_linear[k,2]*curvature[k] - 1)
            # A_m[3,4,k]= (dt*cos(z_linear[k,3])*curvature[k])/(z_linear[k,2]*curvature[k] - 1)
            # A_m[3,5,k]= -(dt*sin(z_linear[k,3])*curvature[k])/(z_linear[k,2]*curvature[k] - 1)
            # A_m[3,6,k]= dt
            # A_m[4,1,k]= 0
            # A_m[4,2,k]= 0
            # A_m[4,3,k]= 0
            # A_m[4,4,k]= 1 - c_f*dt
            # A_m[4,5,k]= dt*z_linear[k,6]
            # A_m[4,6,k]= dt*z_linear[k,5]
            # A_m[5,1,k]= 0
            # A_m[5,2,k]= 0
            # A_m[5,3,k]= 0
            # A_m[5,4,k]= -dt*(z_linear[k,6] + ((B*C*(-g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])))*(z_linear[k,5] - L_a*z_linear[k,6]))/(2*z_linear[k,4]^2*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] -L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) + (B*C*(-g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2])*(z_linear[k,5] + L_a*z_linear[k,6]))/(2*z_linear[k,4]^2*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)))/m)
            # A_m[5,5,k]= (dt*((B*C*(-g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4]))))/(2*z_linear[k,4]*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] - L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) + (B*C*(-g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*z_linear[k,4]*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1))))/m + 1
            # A_m[5,6,k]= -dt*(z_linear[k,4] + ((B*C*L_a*(-g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4]))))/(2*z_linear[k,4]*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] - L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) - (B*C*L_a*(-g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*z_linear[k,4]*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)))/m)
            # A_m[6,1,k]= 0
            # A_m[6,2,k]= 0
            # A_m[6,3,k]= 0
            # A_m[6,4,k]= -(dt*((B*C*L_a*(g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])))*(z_linear[k,5] - L_a*z_linear[k,6]))/(2*z_linear[k,4]^2*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] - L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) - (B*C*L_a*(g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2])*(z_linear[k,5] + L_a*z_linear[k,6]))/(2*z_linear[k,4]^2*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1))))/I_z
            # A_m[6,5,k]= (dt*((B*C*L_a*(g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4]))))/(2*z_linear[k,4]*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] - L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) - (B*C*L_a*(g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*z_linear[k,4]*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1))))/I_z
            # A_m[6,6,k]= -(dt*((B*C*L_a^2*(g*m*mu)*cos(C*atan(B*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4]))))/(2*z_linear[k,4]*(B^2*atan((z_linear[k,5] - L_a*z_linear[k,6])/z_linear[k,4])^2 + 1)*((z_linear[k,5] - L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1)) + (B*C*L_a^2*(g*m*mu)*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*z_linear[k,4]*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1)*((z_linear[k,5] + L_a*z_linear[k,6])^2/z_linear[k,4]^2 + 1))))/I_z +1
            # B_m[1,1,k]= 0
            # B_m[1,2,k]= 0
            # B_m[2,1,k]= 0
            # B_m[2,2,k]= 0
            # B_m[3,1,k]= 0
            # B_m[3,2,k]= 0
            # B_m[4,1,k]= dt
            # B_m[4,2,k]= 0
            # B_m[5,1,k]= 0
            # B_m[5,2,k]= -(dt*((g*m*mu*sin(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*sin(u_linear[k,2]))/2 - (B*C*g*m*mu*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1))))/m
            # B_m[6,1,k]= 0
            # B_m[6,2,k]= -(dt*((L_a*g*m*mu*sin(u_linear[k,2])*sin(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4])))))/2 - (B*C*L_a*g*m*mu*cos(C*atan(B*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))))*cos(u_linear[k,2]))/(2*(B^2*(u_linear[k,2] - atan((z_linear[k,5] + L_a*z_linear[k,6])/z_linear[k,4]))^2 + 1))))/I_z
        end
        setvalue(mdl.A,A_m)
        setvalue(mdl.B,B_m)
        
        setvalue(mdl.selStates,agent.SS.selStates)
        setvalue(mdl.stateCost,agent.SS.stateCost)
        if agent.mpcParams.delay_df > 0
            setvalue(mdl.df_his,agent.mpcSol.df_his)
        end
        # if agent.mpcParams.delay_a > 0
        #     setvalue(mdl.a_his, agent.mpcSol.a_his)
        # end
        setvalue(mdl.uPrev,agent.mpcSol.u_prev)
        
        agent.mpcSol.sol_status  = solve(mdl.mdl)
        agent.mpcSol.u = getvalue(mdl.u_Ol) + u_linear
        agent.mpcSol.z = vcat(z_linear[1,:], getvalue(mdl.z_Ol) + z_linear[2:end,:])

        agent.mpcSol.a_x = agent.mpcSol.u[1+agent.mpcParams.delay_a,1] 
        agent.mpcSol.d_f = agent.mpcSol.u[1+agent.mpcParams.delay_df,2]
        agent.cmd.motor  = agent.mpcSol.a_x 
        agent.cmd.servo  = agent.mpcSol.d_f 

        # println(agent.mpcSol.u)
        # println(agent.mpcSol.z)

        push!(agent.mpcSol.a_his,agent.mpcSol.a_x)
        shift!(agent.mpcSol.a_his)
        push!(agent.mpcSol.df_his,agent.mpcSol.d_f)
        shift!(agent.mpcSol.df_his)        
    end
end # end of the module: solveMpcProblem