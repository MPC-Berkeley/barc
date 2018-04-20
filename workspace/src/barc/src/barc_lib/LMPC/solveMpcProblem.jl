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

function solveMpcProblem_pathFollow(mdl::MpcModel_pF,mpcParams_pF::MpcParams,modelParams::ModelParams,
                                    z_curr::Array{Float64,1},
                                    z_prev::Array{Float64,2},u_prev::Array{Float64,2},track::Track)

    # Load Parameters
    v_ref=mpcParams_pF.vPathFollowing
    width=0.8*0.9 # 0.9 is to give some margin for safety due tp hard constraint for simple path following case
    

    z_final=car_sim_kin(z_prev[end,:],u_prev[end,:],track,modelParams)
    z_curvature=vcat(z_curr',z_prev[3:end,:],z_final)
    # println("u_prev is $(u_prev[:,1])")
    # println("s is $(z_curvature[:,1])")
    curvature=curvature_prediction(z_curvature,track)
    # println("predicted curvature is $(curvature)")
    ey_ref=zeros(mpcParams_pF.N+1)
    for i=1:mpcParams_pF.N+1
        ey_ref[i]=-curvature[i]/track.max_curvature*width/2
    end
    z_ref = hcat(zeros(mpcParams_pF.N+1,1),ey_ref,zeros(mpcParams_pF.N+1,1),v_ref*ones(mpcParams_pF.N+1,1))
    # println("z_ref is: $z_ref")

    # Update current initial condition, curvature and previous input
    setvalue(mdl.z0,z_curr)
    setvalue(mdl.c,curvature)
    setvalue(mdl.uPrev,u_prev)
    setvalue(mdl.z_Ref,z_ref)

    # Solve Problem and return solution
    sol_status  = solve(mdl.mdl)
    sol_u       = getvalue(mdl.u_Ol)
    sol_z       = getvalue(mdl.z_Ol)
    println("Solved, status = $sol_status")
    return sol_z,sol_u,sol_status
end

function solveMpcProblem_featureData(mdl::MpcModel_pF,mpcParams_pF::MpcParams,modelParams::ModelParams,
                                     z_curr::Array{Float64,1},
                                     z_prev::Array{Float64,2},u_prev::Array{Float64,2},track::Track,v_ref::Float64)

    z_final=car_sim_kin(z_prev[end,:],u_prev[end,:],track,modelParams)
    # println("z_curr is $z_curr")
    # println("z_prev is $z_prev")
    # println("z_final is $z_final")
    z_curvature=vcat(z_curr',z_prev[3:end,:],z_final)

    curvature=curvature_prediction(z_curvature,track)
    z_ref = hcat(zeros(mpcParams_pF.N+1,3),v_ref*ones(mpcParams_pF.N+1,1))

    # Update current initial condition, curvature and previous input
    setvalue(mdl.z0,z_curr)
    setvalue(mdl.c,curvature)
    setvalue(mdl.uPrev,u_prev)
    setvalue(mdl.z_Ref,z_ref)

    # Solve Problem and return solution
    sol_status  = solve(mdl.mdl)
    sol_u       = getvalue(mdl.u_Ol)
    sol_z       = getvalue(mdl.z_Ol)
    # println("Solved, status = $sol_status")
    return sol_z,sol_u,sol_status
end

function solveMpcProblem_convhull_dyn_iden(mdl::MpcModel_convhull_dyn_iden,mpcParams::MpcParams,
                                           mpcCoeff::MpcCoeff,lapStatus::LapStatus,zCurr::Array{Float64,1},
                                           zPrev::Array{Float64,2},uPrev::Array{Float64,2},selectedStates::SelectedStates,track::Track)

   # warm start: Julia will do the warm start automatically
   if lapStatus.switchingLap
       println("warm start during switching")
       # zPrev[:,1]-=track.s
       setvalue(mdl.z_Ol,vcat(zPrev[2:end,:],zPrev[end,:]'))
       setvalue(mdl.u_Ol,vcat(uPrev[2:end,:],uPrev[end,:]'))
       lapStatus.switchingLap=false
   end
   # IMPORTANT: do not comment out this warm-start setiing
   setvalue(mdl.z_Ol,vcat(zPrev[2:end,:],zPrev[end,:]'))
   setvalue(mdl.u_Ol,vcat(uPrev[2:end,:],uPrev[end,:]'))
   # zeros in the model initialization is dangerous for optimization: invalid number might occur when divided by zero happends

   selStates       = selectedStates.selStates
   statesCost      = selectedStates.statesCost

   z_curvature=vcat(z_curr',z_prev[3:end,:])
   curvature=curvature_prediction(z_curvature,track)

   # Update current initial condition, curvature and System ID coefficients
   setvalue(mdl.z0,zCurr)
   setvalue(mdl.uPrev,uPrev)
   setvalue(mdl.c,curvature)       # Track curvature
   setvalue(mdl.c_Vx,mpcCoeff.c_Vx)         # System ID coefficients
   setvalue(mdl.c_Vy,mpcCoeff.c_Vy)
   setvalue(mdl.c_Psi,mpcCoeff.c_Psi)
   setvalue(mdl.selStates,selStates)
   setvalue(mdl.statesCost,statesCost)

   # Solve Problem and return solution
   sol_status  = solve(mdl.mdl)
   sol_u       = getvalue(mdl.u_Ol)
   sol_z       = getvalue(mdl.z_Ol)
   println("Solved, status = $sol_status")
   return sol_z,sol_u,sol_status
end