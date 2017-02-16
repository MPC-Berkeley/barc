# Variable definitions
# mdl.z_Ol[i,j] = z_OpenLoop, open loop prediction of the state, i = state, j = step

# States in path following mode:
# i = 1 -> s
# i = 2 -> ey
# i = 3 -> epsi
# i = 4 -> v

# States in LMPC and system ID mode:
# i = 1 -> xDot
# i = 2 -> yDot
# i = 3 -> psiDot
# i = 4 -> ePsi
# i = 5 -> eY
# i = 6 -> s

function solveMpcProblem(mdl::MpcModel,mpcSol::MpcSol,mpcCoeff::MpcCoeff,mpcParams::MpcParams,trackCoeff::TrackCoeff,lapStatus::LapStatus,posInfo::PosInfo,modelParams::ModelParams,zCurr::Array{Float64},uPrev::Array{Float64},rhoPrev::Array{Float64})

    # Load Parameters
    sol_status::Symbol
    sol_u::Array{Float64,2}
    sol_z::Array{Float64,2}

    # Update current initial condition, curvature and System ID coefficients
    setvalue(mdl.z0,zCurr)
    setvalue(mdl.uPrev,uPrev)
    setvalue(mdl.rhoPrev,rhoPrev)

    setvalue(mdl.coeff,trackCoeff.coeffCurvature)       # Track curvature
    setvalue(mdl.coeffTermCost,mpcCoeff.coeffCost)      # Terminal cost
    setvalue(mdl.coeffTermConst,mpcCoeff.coeffConst)    # Terminal constraints

    # Solve Problem and return solution
    sol_status  = solve(mdl.mdl)
    sol_u       = getvalue(mdl.u_Ol)
    sol_z       = getvalue(mdl.z_Ol)

    # export data
    mpcSol.a_x = sol_u[1,1]
    mpcSol.d_f = sol_u[1,2]
    mpcSol.phi = sol_u[1,3]
    mpcSol.u   = sol_u
    mpcSol.z   = sol_z
    mpcSol.solverStatus = sol_status
    mpcSol.cost = zeros(8)
    mpcSol.cost = [getobjectivevalue(mdl.mdl),0,getvalue(mdl.derivCost),getvalue(mdl.controlCost),getvalue(mdl.modelErrorCost),getvalue(mdl.costZTerm),getvalue(mdl.constZTerm),getvalue(mdl.laneCost)]
    mpcSol.ParInt = getvalue(mdl.ParInt)


    println("Solved, status = $sol_status")
    nothing
end

function solveMpcProblem_pathFollow(mdl::MpcModel_pF,mpcSol::MpcSol,mpcParams::MpcParams,trackCoeff::TrackCoeff,posInfo::PosInfo,modelParams::ModelParams,zCurr::Array{Float64},uPrev::Array{Float64})

    # Load Parameters
    coeffCurvature  = trackCoeff.coeffCurvature::Array{Float64,1}

    sol_status::Symbol
    sol_u::Array{Float64,2}
    sol_z::Array{Float64,2}

    # Update current initial condition, curvature and previous input
    setvalue(mdl.z0,zCurr)
    setvalue(mdl.uPrev,uPrev)
    setvalue(mdl.coeff,coeffCurvature)

      

    # Solve Problem and return solution
    sol_status  = solve(mdl.mdl)
    sol_u       = getvalue(mdl.u_Ol)
    sol_z       = getvalue(mdl.z_Ol)

    mpcSol.a_x = sol_u[1,1]
    mpcSol.d_f = sol_u[1,2]
    mpcSol.phi = 0.0
    mpcSol.u   = sol_u
    mpcSol.z   = sol_z
    mpcSol.solverStatus = sol_status
    mpcSol.cost = zeros(8)

    println("Solved, status = $sol_status")
    
    nothing
end
