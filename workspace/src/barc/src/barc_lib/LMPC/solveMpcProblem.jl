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
    z_curvature=vcat(z_curr',z_prev[3:end,:],z_final')

    curvature=curvature_prediction(z_curvature,track)
    ey_ref=zeros(mpcParams_pF.N+1)
    for i=1:mpcParams_pF.N+1
        ey_ref[i]=-curvature[i]/track.max_curvature*width/2
    end
    z_ref = hcat(zeros(mpcParams_pF.N+1,1),ey_ref,zeros(mpcParams_pF.N+1,1),v_ref*ones(mpcParams_pF.N+1,1))

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