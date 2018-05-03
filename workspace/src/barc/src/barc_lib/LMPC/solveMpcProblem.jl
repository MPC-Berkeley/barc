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

function solveMpcProblem_pathFollow(mdl::MpcModel_pF,mpcParams_pF::MpcParams,modelParams::ModelParams,mpcSol::MpcSol,
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
    setvalue(mdl.df_his,mpcSol.df_his)
    setvalue(mdl.z_Ref,z_ref)

    # Solve Problem and return solution
    sol_status  = solve(mdl.mdl)
    sol_u       = getvalue(mdl.u_Ol)
    sol_z       = getvalue(mdl.z_Ol)
    # INPUT DELAY HISTORY UPDATE
    mpcSol.df_his[1:end-1] = mpcSol.df_his[2:end]
    mpcSol.df_his[end] = sol_u[2,2]
    # println("Solved, status = $sol_status")
    return sol_z,sol_u,sol_status
end

function solveMpcProblem_featureData(mdl::MpcModel_pF,mpcParams_pF::MpcParams,modelParams::ModelParams,mpcSol::MpcSol,
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
    setvalue(mdl.df_his,mpcSol.df_his)
    setvalue(mdl.uPrev,u_prev)
    setvalue(mdl.z_Ref,z_ref)

    # Solve Problem and return solution
    sol_status  = solve(mdl.mdl)
    sol_u       = getvalue(mdl.u_Ol)
    sol_z       = getvalue(mdl.z_Ol)
    # println("Solved, status = $sol_status")
    return sol_z,sol_u,sol_status
end

function solveMpcProblem_convhull_dyn_iden(mdl::MpcModel_convhull_dyn_iden,mpcParams::MpcParams,mpcSol::MpcSol,
                                           mpcCoeff::MpcCoeff,lapStatus::LapStatus,zCurr::Array{Float64,1},
                                           zPrev::Array{Float64,2},uPrev::Array{Float64,2},selectedStates::SelectedStates,track::Track)

   # warm start: Julia will do the warm start automatically
   # if lapStatus.switchingLap
   #     println("warm start during switching")
   #     # zPrev[:,1]-=track.s
   #     setvalue(mdl.z_Ol,vcat(zPrev[2:end,:],zPrev[end,:]'))
   #     setvalue(mdl.u_Ol,vcat(uPrev[2:end,:],uPrev[end,:]'))
   #     lapStatus.switchingLap=false
   # end
   # IMPORTANT: this warm start must be done manually when swiching the lap, but here, this warm start is done in the swiching lap section outside
   setvalue(mdl.z_Ol,vcat(zPrev[2:end,:],zPrev[end,:]))
   setvalue(mdl.u_Ol,vcat(uPrev[2:end,:],uPrev[end,:]))
   
   # zeros in the model initialization is dangerous for optimization: invalid number might occur when divided by zero happends

   selStates       = selectedStates.selStates
   statesCost      = selectedStates.statesCost

   z_curvature=vcat(zCurr',zPrev[3:end,:])
   curvature=curvature_prediction(z_curvature,track)

   # Update current initial condition, curvature and System ID coefficients
   setvalue(mdl.z0,zCurr)
   setvalue(mdl.uPrev,uPrev)
   setvalue(mdl.df_his,mpcSol.df_his)
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
   # INPUT DELAY HISTORY UPDATE
   mpcSol.df_his[1:end-1] = mpcSol.df_his[2:end]
   mpcSol.df_his[end] = sol_u[2,2]
   println("Solved, status = $sol_status")
   return sol_z,sol_u,sol_status
end

function solveMpcProblem_convhull_kin_linear(mdl::MpcModel_convhull_kin_linear,mpcParams::MpcParams,modelParams::ModelParams,
                                             lapStatus::LapStatus,
                                             z_linear::Array{Float64,2},u_linear::Array{Float64,2},
                                             zPrev::Array{Float64,2},uPrev::Array{Float64,2}, # this is the delta state hot start
                                             selectedStates::SelectedStates,track::Track)

    # Interface: from absolute value to relative state value
    n_state=4; n_input=2;
    # warm start: Julia will do the warm start automatically
    # if lapStatus.switchingLap
    #     println("warm start during switching")
    #     # setvalue(mdl.z_Ol,zPrev[2:end,:])
    #     # setvalue(mdl.u_Ol,vcat(uPrev[2:end,:],uPrev[end,:]'))
    #     lapStatus.switchingLap=false
    # end

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
    # println("Solved, status = $sol_status")
    return sol_z,sol_u,sol_status
end