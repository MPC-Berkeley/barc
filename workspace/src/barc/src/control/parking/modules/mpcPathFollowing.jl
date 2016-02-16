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
        println("Starting creation of model .....")
        m = new()
        
        # get model parameters
        Ts          = mdlParams.Ts
        L           = mdlParams.L

        # get mpc parameters
        N           = mpcParams.N
        Q           = mpcParams.Q
        R           = mpcParams.R
        
        # Create Model
        println("Creating model .....")
        mdl = Model(solver = IpoptSolver(print_level=0,max_cpu_time=5.10))
        #,linear_solver="ma57",print_user_options="yes"))

        # Create variables (these are going to be optimized)
        @variable( mdl, zOL[1:(N+1),1:4], start = 0)       # z = x, y, psi, v
        @variable( mdl, uOL[1:N,1:2], start = 0)           # u = df, acc

        # Set bounds on inputs
        println("Setting bounds .....")
        u_lb_4s = ones(mpcParams.N,1) * [-1  -0.5]                        # lower bounds [steering, acceleration]
        u_ub_4s = ones(mpcParams.N,1) * [1   0.5]                         # upper bounds

        for i=1:2
            for j=1:N
                setlowerbound(uOL[j,i], u_lb_4s[j,i])
                setupperbound(uOL[j,i], u_ub_4s[j,i])
            end
        end

        println("Initializing parameters .....")
        @NLparameter(mdl, z0[i=1:4] == 0)
        @NLparameter(mdl, zRef[i=1:N+1,j=1:4] == 0)
        @NLparameter(mdl, uRef[i=1:N,j=1:2] == 0)

        # System dynamics
        println("Setting initial condition .....")
        @NLconstraint(mdl, [i=1:4], zOL[1,i] == z0[i])         # initial condition
        
        println("Defining systems dynamic constraints .....")
        for i=1:N
            @NLconstraint(mdl, zOL[i+1,1] == zOL[i,1] + Ts*(zOL[i,4]*cos( zOL[i,3] ))  )                # x
            @NLconstraint(mdl, zOL[i+1,2] == zOL[i,2] + Ts*(zOL[i,4]*sin( zOL[i,3] ))  )                # y
            @NLconstraint(mdl, zOL[i+1,3] == zOL[i,3] + Ts*(zOL[i,4]*tan( uOL[i,1] )/L)  )              # psi
            @NLconstraint(mdl, zOL[i+1,4] == zOL[i,4] + Ts*(uOL[i,2]))                                  # v
        end

        # Cost definitions
        println("Defining cost functions .....")
        # Input cost
        # ---------------------------------
        @NLexpression(mdl, costU, 0.5*sum(R[i,i] * sum((uOL[j,i] - uRef[j,i])^2 for j=1:N) for i = 1:2))

        # State cost
        # ---------------------------------
        @NLexpression(mdl, costZ, 0.5*sum(Q[i,i] * sum((zOL[j,i] - zRef[j,i])^2 for j=2:N+1) for i = 1:4))

        # Objective function
        @NLobjective(mdl, Min, costZ + costU)
        #@NLobjective(mdl, Min, costZ)

        # create first artificial solution (for warm start)
        println("Creating first artificial solution .....")
        for i=1:N+1
            setvalue(zOL[i,:],[0,0,0,0])
        end
        for i=1:N
            setvalue(uOL[i,:],[0.0,0.0])
        end
        
        # First solve
        println("Attemping first solve  .....")
        sol_stat=solve(mdl)
        println("Finished solve 1: $sol_stat")
        sol_stat=solve(mdl)
        println("Finished solve 2: $sol_stat")
        
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

    mpcSol.df  = sol_u[1,1]
    mpcSol.acc = sol_u[1,2]
    mpcSol.uOL   = sol_u
    mpcSol.zOL   = sol_z
    mpcSol.solverStatus = sol_status
end

#=
function InitParams(mpcParams::MpcParams, modelParams::MdlParams)
    mpcParams.N                 = 50
    mpcParams.Q                 = [10.0, 10.0, 1.0, 1.0]
    mpcParams.R                 = [1.0, 1.0]                            

    modelParams.L               = 2.7
    modelParams.Ts              = 0.1   
end
=#