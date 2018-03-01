[1mdiff --git a/workspace/src/barc/src/barc_lib/LMPC/MPC_models.jl b/workspace/src/barc/src/barc_lib/LMPC/MPC_models.jl[m
[1mindex b33c103..be58230 100644[m
[1m--- a/workspace/src/barc/src/barc_lib/LMPC/MPC_models.jl[m
[1m+++ b/workspace/src/barc/src/barc_lib/LMPC/MPC_models.jl[m
[36m@@ -540,11 +540,11 @@[m [mtype MpcModel_convhull[m
         #     @NLconstraint(mdl, u_Ol[i+1,1]-u_Ol[i,1] >= -0.2)[m
         # end[m
 [m
[31m-        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] <= 0.06)[m
[31m-        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] >= -0.06)[m
[32m+[m[32m        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] <= 0.08)[m
[32m+[m[32m        @NLconstraint(mdl, u_Ol[1,2]-uPrev[1,2] >= -0.08)[m
         for i=1:N-1 # Constraints on u:[m
[31m-            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] <= 0.06)[m
[31m-            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] >= -0.06)[m
[32m+[m[32m            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] <= 0.08)[m
[32m+[m[32m            @NLconstraint(mdl, u_Ol[i+1,2]-u_Ol[i,2] >= -0.08)[m
         end[m
 [m
        [m
[1mdiff --git a/workspace/src/barc/src/barc_lib/LMPC/functions.jl b/workspace/src/barc/src/barc_lib/LMPC/functions.jl[m
[1mindex e4e90fb..ae130af 100644[m
[1m--- a/workspace/src/barc/src/barc_lib/LMPC/functions.jl[m
[1m+++ b/workspace/src/barc/src/barc_lib/LMPC/functions.jl[m
[36m@@ -68,9 +68,9 @@[m [mfunction InitializeParameters(mpcParams::MpcParams,mpcParams_pF::MpcParams,track[m
         mpcParams.Q_term_cost       = 3                        # scaling of Q-function[m
         mpcParams.delay_df          = 3                             # steering delay[m
         mpcParams.delay_a           = 1                             # acceleration delay[m
[31m-        mpcParams.Q_lane            = 1                      # weight on the soft constraint for the lane[m
[32m+[m[32m        mpcParams.Q_lane            = 2                      # weight on the soft constraint for the lane[m
         mpcParams.Q_vel             = 1                    # weight on the soft constraint for the maximum velocity[m
[31m-        mpcParams.Q_slack           = 1*[5*20.0,20.0,10.0,30.0,80.0,50.0]#[20.0,10.0,10.0,30.0,80.0,50.0]  #vx,vy,psiDot,ePsi,eY,s[m
[32m+[m[32m        mpcParams.Q_slack           = 1*[5*20.0,1*20.0,1*10.0,30.0,80.0,50.0]#[20.0,10.0,10.0,30.0,80.0,50.0]  #vx,vy,psiDot,ePsi,eY,s[m
         mpcParams.Q_obs             = ones(Nl*selectedStates.Np)# weight to esclude some of the old trajectories[m
 [m
     elseif selectedStates.simulator == true  # if the simulator is in use[m
[1mdiff --git a/workspace/src/barc/src/state_estimation_SensorKinematicModel.py b/workspace/src/barc/src/state_estimation_SensorKinematicModel.py[m
[1mindex dc2d986..18c8ad9 100755[m
[1m--- a/workspace/src/barc/src/state_estimation_SensorKinematicModel.py[m
[1m+++ b/workspace/src/barc/src/state_estimation_SensorKinematicModel.py[m
[36m@@ -224,8 +224,8 @@[m [mdef state_estimation():[m
 [m
     Q = diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,dt*qp,0.1, 0.2,0.2,1.0,1.0,0.1])[m
     R = diag([5.0,5.0,1.0,10.0,100.0,1000.0,1000.0,     5.0,5.0,10.0,1.0, 10.0,10.0])[m
[31m-    #R = diag([20.0,20.0,1.0,10.0,100.0,1000.0,1000.0,     20.0,20.0,10.0,1.0, 10.0,10.0])[m
[31m-    #         x,y,v,psi,psiDot,a_x,a_y, x, y, psi, v[m
[32m+[m[32m    R = diag([4*5.0,4*5.0,1.0,10.0,2*100.0,2*1000.0,2*1000.0,     4*5.0,4*5.0,10.0,1.0, 2*10.0,10.0])[m
[32m+[m[32m    #             x,   y,  v, psi,psiDot,a_x,a_y, x, y, psi, v[m
 [m
     # Set up track parameters[m
     l = Localization()[m
