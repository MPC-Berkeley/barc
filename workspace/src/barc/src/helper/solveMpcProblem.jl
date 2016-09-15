# Variable definitions
# z_Ol[i,j] = z_OpenLoop, open loop prediction of the state, i = state, j = step

# States:
# i = 1 -> s
# i = 2 -> ey
# i = 3 -> epsi
# i = 4 -> v

function solveMpcProblem(mpcCoeff,mpcParams,trackCoeff,lapStatus,posInfo,modelParams,zCurr,uCurr)

    # Load Parameters
    coeffCurvature  = trackCoeff.coeffCurvature
    N               = mpcParams.N
    Q               = mpcParams.Q
    R               = mpcParams.R
    coeffTermCost   = mpcCoeff.coeffCost
    coeffTermConst  = mpcCoeff.coeffConst
    order           = mpcCoeff.order       # polynomial order of terminal constraints and cost approximation
    s_start         = posInfo.s_start
    s_target        = posInfo.s_target
    ey_max          = trackCoeff.width/2

    QderivZ         = mpcParams.QderivZ
    QderivU         = mpcParams.QderivU

    v_ref           = mpcParams.vPathFollowing
    if lapStatus.currentLap > 1
        #v_ref = 10
    end

    global mdl

    # Create function-specific parameters
    z_Ref           = cat(1,s_target*ones(1,N+1),zeros(2,N+1),v_ref*ones(1,N+1))       # Reference trajectory: path following -> stay on line and keep constant velocity
    u_Ref           = zeros(2,N)

    # Update current initial condition
    setvalue(z0,zCurr)
    # Update curvature
    setvalue(coeff,coeffCurvature)

    @NLexpression(mdl, costZ,       0)
    @NLexpression(mdl, costZTerm,   0)
    @NLexpression(mdl, constZTerm,  0)
    @NLexpression(mdl, derivCost,   0)
    @NLexpression(mdl, laneCost,    0)

    # Derivative cost
    # ---------------------------------
    @NLexpression(mdl, derivCost, 0*sum{QderivZ[j]*((zCurr[j]-z_Ol[j,1])^2+sum{(z_Ol[j,i]-z_Ol[j,i+1])^2,i=1:N}),j=1:4} +
                                  sum{QderivU[j]*((uCurr[j]-u_Ol[j,1])^2+sum{(u_Ol[j,i]-u_Ol[j,i+1])^2,i=1:N-1}),j=1:2})

    # Lane cost
    # ---------------------------------
    @NLexpression(mdl, laneCost, 10*sum{(0.5+0.5*tanh(50*(z_Ol[2,i]-ey_max))) + (0.5-0.5*tanh(50*(z_Ol[2,i]+ey_max))),i=1:N+1})

    # Control Input cost
    # ---------------------------------
    @NLexpression(mdl, controlCost, 0.5*sum{R[j]*sum{(u_Ol[j,i]-u_Ref[j,i])^2,i=1:N},j=1:2})

    # Terminal constraints (soft), starting from 2nd lap
    # ---------------------------------
    if lapStatus.currentLap > 2    # if at least in the 3rd lap
              # termStateErr =       (ParInt*polyval(coeffTermCons[j], nPolyOrderTermCons, ZOlGlobal[Hp*nz+5]) +
              #               + (1 - ParInt)*polyval(coeffTermCons_1[j], nPolyOrderTermCons, ZOlGlobal[Hp*nz+5])) - ZOlGlobal[Hp*nz+j];
        @NLexpression(mdl, constZTerm, 10*(sum{(ParInt*sum{coeffTermConst[j,i,1,1]*z_Ol[1,N+1]^(order+1-i),i=1:order+1}+
                                        (1-ParInt)*sum{coeffTermConst[j,i,1,2]*z_Ol[1,N+1]^(order+1-i),i=1:order+1}-z_Ol[j+1,N+1])^2,j=1:3}))
    elseif lapStatus.currentLap == 2        # if in the 2nd lap
        # termStateErr =       polyval(coeffTermCons[j], nPolyOrderTermCons, ZOlGlobal[Hp*nz+5]) - ZOlGlobal[Hp*nz+j];
        @NLexpression(mdl, constZTerm, 10*sum{(sum{coeffTermConst[j,i,1,1]*z_Ol[1,N+1]^(order+1-i),i=1:order+1}-z_Ol[j+1,N+1])^2,j=1:3})
    end

    # Terminal cost
    # ---------------------------------
    if zCurr[1] + s_start > s_target                # if finish line crossed
        println("Crossed finish line. I should not be here.")
        @NLexpression(mdl, costZTerm, 0)            # terminal cost is zero
    else
        if lapStatus.currentLap > 2     # if at least in the 3rd lap
            # costZTerm =     ParInt*polyval(coeffTermCost, nPolyOrderTermCost, ZOlGlobal[Hp*nz+5]) +
            #            + (1-ParInt)*polyval(coeffTermCost_1, nPolyOrderTermCost, ZOlGlobal[Hp*nz+5]);
            @NLexpression(mdl, costZTerm, ParInt*sum{coeffTermCost[i,1,1]*z_Ol[1,N+1]^(order+1-i),i=1:order+1}+
                                      (1-ParInt)*sum{coeffTermCost[i,1,2]*z_Ol[1,N+1]^(order+1-i),i=1:order+1})
            #@NLexpression(mdl, costZTerm, costZTerm_h * (0.5-0.5*tanh(50*(z_Ol[1,N+1]+s_start-s_target))))
                # line above not necessary since the polynomial goes to zero anyways!
        elseif lapStatus.currentLap == 2         # if we're in the second second lap
            # costZTerm =     polyval(coeffTermCost, nPolyOrderTermCost, ZOlGlobal[Hp*nz+5]);
            @NLexpression(mdl, costZTerm, sum{coeffTermCost[i,1,1]*z_Ol[1,N+1]^(order+1-i),i=1:order+1})
            #@NLexpression(mdl, costZTerm, costZTerm_h * (0.5-0.5*tanh(50*(z_Ol[1,N+1]+s_start-s_target))))
        end
    end

    # State cost
    # ---------------------------------
    if lapStatus.currentLap == 1 || zCurr[1] + s_start > s_target      # if we're in the first lap or crossed finish line, just do path following
        # currCostZ = 0.5*Q[j]*pow(ZOlGlobal[(i+1)*nz+j]-ZRefGlobal[(i+1)*nz+j],2);
        if lapStatus.currentLap > 1
            println("I should not be here.")
        end
        @NLexpression(mdl, costZ, 0.5*sum{Q[i]*sum{(z_Ol[i,j]-z_Ref[i,j])^2,j=1:N+1},i=1:4})    # Follow trajectory
        #println(z_Ref)
    else
        #@NLexpression(mdl, costZ_h, 0)          # zero state cost after crossing the finish line
        #@NLexpression(mdl, costZ, 1 + (costZ_h-1) * (0.5+0.5*tanh(50*(z_Ol[1,N+1]+s_start-s_target))))
        @NLexpression(mdl, costZ, 1)
        #@NLexpression(mdl, costZ, 1)
    end

    @NLobjective(mdl, Min, costZ + costZTerm + constZTerm + derivCost + controlCost + laneCost)

    # Solve Problem and return solution
    sol_status  = solve(mdl)
    sol_u       = getvalue(u_Ol)
    mpcSol      = MpcSol(sol_u[1,1],sol_u[2,1],sol_status,getvalue(u_Ol),getvalue(z_Ol),[getvalue(costZ),getvalue(costZTerm),getvalue(constZTerm),getvalue(derivCost),getvalue(controlCost),getvalue(laneCost)])
    #mpcSol      = MpcSol(sol_u[1,1],sol_u[2,1]) # Fast version without logging
    #println(getvalue(costZTerm))
    #println(getvalue(z_Ol[1,N+1]))
    #println(getvalue(constZTerm))
    #println("ParInt = $(getvalue(ParInt))")
    #println("u = $(sol_u[:,1])")

    # if lapStatus.currentLap > 100
    #     ss = collect(zCurr[1]:.01:zCurr[1]+0.3)
    #     #p  = [ss.^4 ss.^3 ss.^2 ss.^1 ss.^0] *coeffTermCost[:,:,1]
    #     p  = [ss.^4 ss.^3 ss.^2 ss.^1 ss.^0] *coeffTermConst[:,:,1,1]
    #     plot(ss,p,getvalue(z_Ol[1,N+1]),getvalue(costZTerm),"o")
    #     grid()
    #     readline()
    # end

    # println(getvalue(z_Ol))
    # println("==============")
    # println(getvalue(u_Ol))
    return mpcSol
end
