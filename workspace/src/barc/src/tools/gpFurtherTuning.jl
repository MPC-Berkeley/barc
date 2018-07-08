#=
This file is used to do GPS validation and further tuning
=#

# folder_name         = "simulations"
folder_name         = "experiments"
result_file_name    = "LMPC-SYS_ID_TI-2018-07-07-10:42"
gp_result_file_name = "GP-SYS_ID_TI-RESULT-2018-07-07-10:42"
gp_feature_file_name= "GP-SYS_ID_TI"
gpStateNum = 4

using JLD
using PyPlot
include("gpLibrary.jl")

# LOADING EXPERIMENT GPR DATA
data = load("$(homedir())/$(folder_name)/$(gp_result_file_name).jld")
erro_s_his      = data["erro_s_his"]
erro_ey_his     = data["erro_ey_his"]
erro_epsi_his   = data["erro_epsi_his"]
erro_vx_his     = data["erro_vx_his"]
erro_vy_his     = data["erro_vy_his"]
erro_psiDot_his = data["erro_psiDot_his"]
GP_vx_his       = data["GP_vx_his"]
GP_vy_his       = data["GP_vy_his"]
GP_psiDot_his   = data["GP_psiDot_his"]

# LOADING DATA TO DO FURTHER GPR TUNING
data = load("$(homedir())/$(folder_name)/$(result_file_name).jld")
z_his   = data["z"]
u_his   = data["u"]
z = zeros(1,gpStateNum)
u = zeros(1,2)
cost = data["cost"]
for i = 5:length(cost)
  if gpStateNum==4
    z = vcat(z,reshape(z_his[1:Int(cost[i]),i,1,1:4],Int(cost[i]),4))
  elseif gpStateNum==6
    z = vcat(z,reshape(z_his[1:Int(cost[i]),i,1,1:6],Int(cost[i]),6))
  end
    u = vcat(u,reshape(u_his[1:Int(cost[i]),i,1,:],Int(cost[i]),2))
end
data = load("$(homedir())/$(folder_name)/$(gp_feature_file_name).jld")
num_sps  	= 9
e_vx        = data["e_vx"]
e_vy        = data["e_vy"]
e_psiDot  	= data["e_psiDot"]
e_vx        = e_vx[1:num_sps:end]
e_vy        = e_vy[1:num_sps:end]
e_psiDot    = e_psiDot[1:num_sps:end]
feature_GP_z= data["z"]
feature_GP_u= data["u"]
feature_GP_z= feature_GP_z[1:num_sps:end,:]
feature_GP_u= feature_GP_u[1:num_sps:end,:]
feature_GP  = hcat(feature_GP_z,feature_GP_u)
if gpStateNum==4
    prep_vx     = gpPrepKin(e_vx,feature_GP)
elseif gpStateNum==6
    prep_vx     = gpPrepDyn(e_vx,feature_GP)
    prep_vy     = gpPrepDyn(e_vy,feature_GP)
    prep_psiDot = gpPrepDyn(e_psiDot,feature_GP)
end

GP_vx_check     = zeros(size(z,1))
GP_vy_check     = zeros(size(z,1))
GP_psiDot_check = zeros(size(z,1))
for i = 1:size(z,1)
    if gpStateNum==4
       GP_vx_check[i]       = gpFullKin(z[i,:],u[i,:],feature_GP,prep_vx)
    elseif gpStateNum==6
       GP_vx_check[i]       = gpFullDyn(z[i,:],u[i,:],feature_GP,prep_vx)
       GP_vy_check[i]       = gpFullDyn(z[i,:],u[i,:],feature_GP,prep_vy)
       GP_psiDot_check[i]   = gpFullDyn(z[i,:],u[i,:],feature_GP,prep_psiDot)
    end
end

# EXPERIMENT GPR RESULT AND FURTHER TUNING RESULT PLOTTING
figure("GPR further tuning: $(file_name)")
subplot(3,2,1); plot(erro_s_his,     "-",alpha=0.5) 
subplot(3,2,2); plot(erro_ey_his,    "-",alpha=0.5)
subplot(3,2,3); plot(erro_epsi_his,  "-",alpha=0.5)
subplot(3,2,4); plot(erro_vx_his,    "-",alpha=0.5); plot(GP_vx_his[:,1],    "o",alpha=0.3);   plot(GP_vx_check[:,1],    "<",alpha=0.3)
subplot(3,2,5); plot(erro_vy_his,    "-",alpha=0.5); plot(GP_vy_his[:,1],    "o",alpha=0.3);   plot(GP_vy_check[:,1],    "<",alpha=0.3)
subplot(3,2,6); plot(erro_psiDot_his,"-",alpha=0.5); plot(GP_psiDot_his[:,1],"o",alpha=0.3);   plot(GP_psiDot_check[:,1],"<",alpha=0.3)
show()