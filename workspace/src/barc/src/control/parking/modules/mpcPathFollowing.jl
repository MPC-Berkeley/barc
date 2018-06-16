#!/usr/bin/env julia

type MdlParams
    L::Float64    
    Ts::Float64
    MdlParams(L=1.0,Ts=0.1) = new(L,Ts)
end

type MpcParams          # parameters for MPC solver
    N::Int64
    Q::Array{Float64,2}
    R::Array{Float64,2}
    MpcParams(N=0,Q=Float64[],R=Float64[]) = new(N,Q,R)
end

type MpcSol             # MPC solution output
    acc::Float64
    df::Float64
    uOL::Array{Float64}
    zOL::Array{Float64}
    J::Array{Float64}
    solverStatus::Symbol
    MpcSol(acc=0.0,df=0.0,uOL=Float64[],zOL=Float64[],J=Float64[],solverStatus=Symbol()) = new(acc,df,uOL,zOL,J,solverStatus)
end

type MpcModel
    z0::Array{JuMP.NonlinearParameter,1}
    zRef::Array{JuMP.NonlinearParameter,2}
    uRef::Array{JuMP.NonlinearParameter,2}
    zOL::Array{JuMP.Variable,2}
    uOL::Array{JuMP.Variable,2}
    costZ::JuMP.NonlinearExpression
    costU::JuMP.NonlinearExpression
    mdl::JuMP.Model

    function MpcModel(mpcParams::MpcParams, mdlParams::MdlParams)
        println("Building mpc controller .....")
        m = new()
        
        # get model parameters
        Ts          = mdlParams.Ts
        L           = mdlParams.L

        # get mpc parameters
        N           = mpcParams.N
        Q           = mpcParams.Q
        R           = mpcParams.R
        
        # Create Model
        mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=5.10))

        # Create variables (these are going to be optimized)
        @variable( mdl, zOL[1:4, 1:(N+1)],  start = 0)       # z = x, y, psi, v
        @variable( mdl, uOL[1:2, 1:N],      start = 0)           # u = df, acc

        # Set bounds on inputs
        u_lb = [-0.6  -0.25]' * ones(1,mpcParams.N)       # lower bounds [steering, acceleration]
        u_ub = [0.6    0.25]' * ones(1,mpcParams.N)       # upper bounds

        for i=1:N
            for j=1:2
                setlowerbound(uOL[j,i], u_lb[j,i])
                setupperbound(uOL[j,i], u_ub[j,i])
            end
        end

        @NLparameter(mdl, z0[i=1:4] == 0)
        @NLparameter(mdl, zRef[j=1:4,i=1:N+1] == 0)
        @NLparameter(mdl, uRef[j=1:2,i=1:N] == 0)

        # System dynamics
        @NLconstraint(mdl, [i=1:4], zOL[i,1] == z0[i])         # initial condition
        
        for i=1:N
            @NLconstraint(mdl, zOL[1,i+1] == zOL[1,i] + Ts*(zOL[4,i]*cos( zOL[3,i] ))  )                # x
            @NLconstraint(mdl, zOL[2,i+1] == zOL[2,i] + Ts*(zOL[4,i]*sin( zOL[3,i] ))  )                # y
            @NLconstraint(mdl, zOL[3,i+1] == zOL[3,i] + Ts*(zOL[4,i]*tan( uOL[1,i] )/L)  )              # psi
            @NLconstraint(mdl, zOL[4,i+1] == zOL[4,i] + Ts*(uOL[2,i]))                                  # v
        end

        # Cost definitions
        # Input cost
        # ---------------------------------
        @NLexpression(mdl, costU, 0.5*sum(R[j,j] * sum((uOL[j,i] - uRef[j,i])^2 for i=1:N) for j = 1:2))

        # State cost
        # ---------------------------------
        @NLexpression(mdl, costZ, 0.5*sum(Q[j,j] * sum((zOL[j,i] - zRef[j,i])^2 for i=2:N+1) for j = 1:4))

        # Objective function
        @NLobjective(mdl, Min, costZ + costU)
        #@NLobjective(mdl, Min, costZ)

        # create first artificial solution (for warm start)
        for i=1:N+1
            setvalue(zOL[:,i],[0.0, 0.0, 0.0, 0.0])
        end
        for i=1:N
            setvalue(uOL[:,i],[0.0, 0.0])
        end
        
        # First solve
        println("Attemping first solve  .....")
        sol_stat = solve(mdl)
        println("Finished solve 1: $sol_stat")
        
        m.mdl   = mdl
        m.z0    = z0
        m.zOL   = zOL
        m.uOL   = uOL
        m.costZ = costZ
        m.costU = costU
        m.zRef  = zRef
        m.uRef  = uRef
        return m
    end
end

function SolveMpcProblem(mdl::MpcModel,mpcSol::MpcSol,zCurr::Array{Float64}, zRef::Array{Float64,2}, uRef::Array{Float64,2})
    # update current initial condition
    setvalue(mdl.z0,   zCurr)
    setvalue(mdl.zRef, zRef)
    setvalue(mdl.uRef, uRef)

    # solve mpc
    sol_status  = solve(mdl.mdl)
    
    # get solution
    sol_u       = getvalue(mdl.uOL)
    sol_z       = getvalue(mdl.zOL)

    mpcSol.df   = sol_u[1,1]
    mpcSol.acc  = sol_u[2,1]
    mpcSol.uOL  = sol_u
    mpcSol.zOL  = sol_z
    mpcSol.solverStatus = sol_status
end