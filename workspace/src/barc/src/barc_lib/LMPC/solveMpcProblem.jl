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

    # println("from decision variable",getvalue(mdl.u_Ol[1:3,2]))
    # println("from nonlinear paramet",getvalue(mdl.df_his))

    # Load Parameters
    v_ref=mpcParams_pF.vPathFollowing
    width=0.8*0.9 # 0.9 is to give some margin for safety due tp hard constraint for simple path following case
    
    z_final=car_sim_kin(z_prev[end,1:4],u_prev[end,:],track,modelParams)
    z_curvature=vcat(z_curr',z_prev[3:end,1:4],z_final)
    # println("u_prev is $(u_prev[:,1])")
    # println("s is $(z_curvature[:,1])")
    curvature=curvature_prediction(z_curvature,track)
    # println("Predicted curvature",round(curvature,2))
    # println("Predicted ey",round(z_curvature[:,2],2))
    # println("Predicted u[2]",round(u_prev[:,2],2))
    # println("predicted curvature is $(curvature)")
    ey_ref=zeros(mpcParams_pF.N+1)
    for i=1:mpcParams_pF.N+1
        ey_ref[i]=-curvature[i]/track.max_curvature*width/2
    end
    # z_ref = hcat(zeros(mpcParams_pF.N+1,1),ey_ref,zeros(mpcParams_pF.N+1,1),v_ref*ones(mpcParams_pF.N+1,1))
    z_ref = hcat(zeros(mpcParams_pF.N+1,1),zeros(mpcParams_pF.N+1),zeros(mpcParams_pF.N+1,1),v_ref*ones(mpcParams_pF.N+1,1))
    

    # println("z_ref is: $z_ref")

    # Update current initial condition, curvature and previous input
    setvalue(mdl.z0,z_curr)
    setvalue(mdl.c,curvature)

    setvalue(mdl.uPrev,u_prev)
    setvalue(mdl.df_his,mpcSol.df_his)
    setvalue(mdl.a_his,mpcSol.a_his)
    # println("a_his",mpcSol.a_his)
    setvalue(mdl.z_Ref,z_ref)

    # Solve Problem and return solution
    sol_status  = solve(mdl.mdl)
    sol_u       = getvalue(mdl.u_Ol)
    sol_z       = getvalue(mdl.z_Ol)
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

function solveMpcProblem_convhull_dyn_iden(mdl::MpcModel_convhull_dyn_iden,mpcSol::MpcSol,
                                           mpcCoeff::MpcCoeff,zCurr::Array{Float64,1},
                                           zPrev::Array{Float64,2},uPrev::Array{Float64,2},selectedStates::SelectedStates,track::Track,
                                           GP_e_vy::Array{Float64,1},GP_e_psidot::Array{Float64,1})

   # IMPORTANT: this warm start must be done manually when swiching the lap, but here, this warm start is done in the swiching lap section outside
   # setvalue(mdl.z_Ol,vcat(zPrev[2:end,:],zPrev[end,:]))
   # setvalue(mdl.u_Ol,vcat(uPrev[2:end,:],uPrev[end,:]))
   
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
   setvalue(mdl.GP_e_vy,GP_e_vy)
   setvalue(mdl.GP_e_psidot,GP_e_psidot)

   # Solve Problem and return solution
   sol_status  = solve(mdl.mdl)
   sol_u       = getvalue(mdl.u_Ol)
   sol_z       = getvalue(mdl.z_Ol)
   # println("Solved, status = $sol_status")
   return sol_z,sol_u,sol_status
end

function solveMpcProblem_convhull_kin_linear(mdl::MpcModel_convhull_kin_linear,mpcSol::MpcSol,mpcParams::MpcParams,modelParams::ModelParams,
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
    # println(size(mdl.df_his))
    # println(size(mpcSol.df_his))
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
    # println("Solved, status = $sol_status")
    return sol_z,sol_u,sol_status
end

function solveMpcProblem_convhull_dyn_linear(mdl::MpcModel_convhull_dyn_linear,mpcSol::MpcSol,mpcParams::MpcParams,modelParams::ModelParams,
                                             lapStatus::LapStatus,
                                             z_linear::Array{Float64,2},u_linear::Array{Float64,2},
                                             zPrev::Array{Float64,2},uPrev::Array{Float64,2}, # this is the delta state hot start
                                             selectedStates::SelectedStates,track::Track,GP_e_vy::Array{Float64,1},GP_e_psidot::Array{Float64,1})

    # Interface: from absolute value to relative state value
    n_state=6; n_input=2;
    # warm start: Julia will do the warm start automatically
    # if lapStatus.switchingLap
    #     println("warm start during switching")
    #     z_linear[:,1]=-=track.s
        # setvalue(mdl.z_Ol,vcat(zPrev[3:end,:],zPrev[end,:]))
        # setvalue(mdl.u_Ol,vcat(uPrev[2:end,:],uPrev[end,:]))
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

    z_linear[1:end-1,5] += GP_e_vy
    z_linear[1:end-1,6] += GP_e_psidot

    setvalue(mdl.z_linear,z_linear)
    setvalue(mdl.u_linear,u_linear)

    # uPrev=u_prev; zPrev=z_prev
    # i=1; j=1; k=1;
    # mdl=mdl_convhull
    L_a=modelParams.l_A; L_b=modelParams.l_B
    dt=modelParams.dt; c_f=modelParams.c_f
    # THOSE TIRE PARAMETERS MIGHT NEED TO BE CHANGED
    m=1.98; I_z=0.03; 
    B = 6.0; C = 1.6; mu = 0.8; g = 9.81
    A_m=zeros(n_state,n_state,mpcParams.N)
    B_m=zeros(n_state,n_input,mpcParams.N)
    curvature=curvature_prediction(z_linear,track)
    for k=1:mpcParams.N
        # curvature=0; curvature_deri=0
        # for i=1:7
        #     curvature = curvature + curvatureCoeff[i]*z_linear[k,1]^(i-1);
        #     curvature_deri = curvature_deri + (i-1)*curvatureCoeff[i]*z_linear[k,1]^(i-2);
        # end
        # A_m[1,1,k]= 1 + dt*(z_linear[k,4]*cos(z_linear[k,3]) - z_linear[k,5]*sin(z_linear[k,3]))*z_linear[k,2]*curvature_deri/(1 - z_linear[k,2]*curvature)^2
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
        # for i=1:n_state
        #     for j=1:n_state
        #         JuMP.fix(mdl.A[i,j,k],A_m[i,j,k])
        #     end
        #     for j=1:n_input
        #         JuMP.fix(mdl.B[i,j,k],B_m[i,j,k])
        #     end
        # end
        setvalue(mdl.A,A_m)
        setvalue(mdl.B,B_m)
    end
    # for k=1:mpcParams.N
    #     for i=1:n_state
    #         for j=1:n_state
    #             A[i,j,k] = subs(mdl.A_s[i,j],mdl.z_s[1]=>z_linear[k,1],mdl.z_s[2]=>z_linear[k,2],
    #                                          mdl.z_s[3]=>z_linear[k,3],mdl.z_s[4]=>z_linear[k,4],
    #                                          mdl.z_s[5]=>z_linear[k,5],mdl.z_s[6]=>z_linear[k,6],
    #                                          mdl.u_s[1]=>u_linear[k,1],mdl.u_s[2]=>u_linear[k,2],
    #                                          mdl.coeff_s[1]=>curvatureCoeff[1],mdl.coeff_s[2]=>curvatureCoeff[2],
    #                                          mdl.coeff_s[3]=>curvatureCoeff[3],mdl.coeff_s[4]=>curvatureCoeff[4],
    #                                          mdl.coeff_s[5]=>curvatureCoeff[5],mdl.coeff_s[6]=>curvatureCoeff[6],
    #                                          mdl.coeff_s[7]=>curvatureCoeff[7])
    #             JuMP.fix(mdl.A[i,j,k],A[i,j,k])
    #         end
    #         for j=1:n_input
    #             B[i,j,k] = subs(mdl.B_s[i,j],mdl.z_s[1]=>z_linear[k,1],mdl.z_s[2]=>z_linear[k,2],
    #                                          mdl.z_s[3]=>z_linear[k,3],mdl.z_s[4]=>z_linear[k,4],
    #                                          mdl.z_s[5]=>z_linear[k,5],mdl.z_s[6]=>z_linear[k,6],
    #                                          mdl.u_s[1]=>u_linear[k,1],mdl.u_s[2]=>u_linear[k,2],
    #                                          mdl.coeff_s[1]=>curvatureCoeff[1],mdl.coeff_s[2]=>curvatureCoeff[2],
    #                                          mdl.coeff_s[3]=>curvatureCoeff[3],mdl.coeff_s[4]=>curvatureCoeff[4],
    #                                          mdl.coeff_s[5]=>curvatureCoeff[5],mdl.coeff_s[6]=>curvatureCoeff[6],
    #                                          mdl.coeff_s[7]=>curvatureCoeff[7])
    #             JuMP.fix(mdl.B[i,j,k],B[i,j,k])
    #         end
    #     end
    # end

    # for i=1:selectedStates.Nl*selectedStates.Np
    #     for j=1:n_state
    #         JuMP.fix(mdl.selStates[i,j],selStates[i,j])
    #     end
    #     JuMP.fix(mdl.statesCost[i],statesCost[i])
    # end
    # println(size(mdl.selStates))
    # println(size(selStates))
    setvalue(mdl.selStates,selStates)
    setvalue(mdl.statesCost,statesCost)

    setvalue(mdl.df_his,mpcSol.df_his)
    # println(getvalue(mdl.df_his))
    # for i=1:n_state
    #     JuMP.fix(mdl.zPrev[i],zPrev[1,i])
    # end
    # JuMP.fix(mdl.uPrev[1],uPrev[1,1])
    # JuMP.fix(mdl.uPrev[2],uPrev[1,2])
    # println(size(mdl.uPrev))
    # println(size(uPrev))
    setvalue(mdl.uPrev,uPrev)
    setvalue(mdl.zPrev,zPrev)

    setvalue(mdl.GP_e_vy,GP_e_vy)
    setvalue(mdl.GP_e_psidot,GP_e_psidot)

    # Solve Problem and return solution
    sol_status  = solve(mdl.mdl)
    sol_u       = getvalue(mdl.u_Ol) + u_linear
    sol_z       = vcat(z_linear[1,:], getvalue(mdl.z_Ol) + z_linear[2:end,:])
    # println("Solved, status = $sol_status")
    return sol_z,sol_u,sol_status
end