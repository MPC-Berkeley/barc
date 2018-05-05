# functions dummy calls
# FUNCTION DUMMY CALLS: this is important to call all the functions that will be used before for initial compiling
data = load("$(homedir())/simulations/oldSS.jld") # oldSS.jld is a dummy data for initialization
oldSS_dummy = data["oldSS"]
lapStatus_dummy = LapStatus(1+selectedStates.Nl,1,false,false,0.3)
selectedStates_dummy=find_SS(oldSS_dummy,selectedStates,z_prev,lapStatus_dummy,modelParams,mpcParams,track)
z = rand(1,6); u = rand(1,2)
(iden_z,iden_u)=find_feature_dist(feature_z,feature_u,z,u)
(c_Vx,c_Vy,c_Psi)=coeff_iden_dist(iden_z,iden_u)
(~,~,~)=solveMpcProblem_convhull_dyn_iden(mdl_convhull,mpcParams,mpcSol,mpcCoeff,lapStatus_dummy,rand(6),rand(mpcParams.N+1,6),rand(mpcParams.N,2),selectedStates_dummy,track)
selectedStates_dummy.selStates=s6_to_s4(selectedStates_dummy.selStates)
(~,~,~)=solveMpcProblem_convhull_kin_linear(mdl_kin_lin,mpcParams_4s,modelParams,lapStatus,rand(mpcParams.N+1,4),rand(mpcParams.N,2),rand(mpcParams.N+1,6),rand(mpcParams.N,2),selectedStates_dummy,track)
(~,~,~)=car_pre_dyn(rand(1,6)+1,rand(mpcParams.N,2),track,modelParams,6)
(~,~,~)=find_SS_dist(solHistory,rand(1,6),rand(1,2),lapStatus)