#=
This file is for GPR playback and tuning on GPR
=#
using JLD
using PyPlot

function gpPrepKin(e::Array{Float64,1},zu::Array{Float64,2})
    # find the closest local feature points
    num = size(zu,1)
    Z=zeros(num,num)
    for i = 1:num
        for j=1:num
            z = zu[i,2:6]-zu[j,2:6]
            Z[i,j] = (10*z[1]^2+10*z[2]^2+z[3]^2+z[4]^2+5*z[5]^2)
        end
    end
    K=0.1^2*exp(-2.0*Z)
    return K\e 
end

function gpFullKin(z::Array{Float64,2},u::Array{Float64,2},feature_state::Array{Float64,2},GP_prepare::Array{Float64,1})
    state = hcat(z,u)
    z = feature_state[:,2:6].-state[1,2:6]
    Z = 10*z[:,1].^2+10*z[:,2].^2+z[:,3].^2+z[:,4].^2+5*z[:,5].^2
    k = 0.1^2*exp(-2.0*Z)
    GP_e = k'*GP_prepare
    return GP_e[1]
end

folder_name = "simulations"
file_name = "LMPC-KIN-2018-06-14-17:36"
data = load("$(homedir())/$(folder_name)/$(file_name).jld")
z_his	= data["z"]
u_his	= data["u"]
# z = Array{Float64}(0,4)
# u = Array{Float64}(0,2)
z = zeros(1,4)
u = zeros(1,2)
cost = data["cost"]
for i = 5:length(cost)
	z = vcat(z,reshape(z_his[1:Int(cost[i]),i,1,1:4],Int(cost[i]),4))
	u = vcat(u,reshape(u_his[1:Int(cost[i]),i,1,:],Int(cost[i]),2))
end
file_name = "GP-KIN-RESULT-2018-06-14-17:36"
data = load("$(homedir())/$(folder_name)/$(file_name).jld")
erro_s_his 		= data["erro_s_his"]
erro_ey_his 	= data["erro_ey_his"]
erro_epsi_his 	= data["erro_epsi_his"]
erro_vx_his 	= data["erro_vx_his"]
erro_vy_his 	= data["erro_vy_his"]
erro_psiDot_his = data["erro_psiDot_his"]
erro_ey_his 	= data["erro_ey_his"]
erro_epsi_his 	= data["erro_epsi_his"]
GP_s_his 		= data["GP_s_his"]
GP_ey_his 		= data["GP_ey_his"]
GP_epsi_his 	= data["GP_epsi_his"]
GP_vx_his 		= data["GP_vx_his"]
GP_vy_his 		= data["GP_vy_his"]
GP_psiDot_his 	= data["GP_psiDot_his"]

data = load("$(homedir())/$(folder_name)/GP-KIN.jld")
num_sps  	= 2
e_ey    	= data["e_ey"]
e_epsi  	= data["e_epsi"]
e_ey		= e_ey[1:num_sps:end]
e_epsi		= e_epsi[1:num_sps:end]
feature_GP_z= data["z"]
feature_GP_u= data["u"]
feature_GP_z= feature_GP_z[1:num_sps:end,:]
feature_GP_u= feature_GP_u[1:num_sps:end,:]
feature_GP  = hcat(feature_GP_z,feature_GP_u)
prep_ey 	= gpPrepKin(e_ey,feature_GP)
prep_epsi 	= gpPrepKin(e_epsi,feature_GP)

GP_ey_check 	= zeros(size(z,1))
GP_epsi_check 	= zeros(size(z,1))
for i = 1:size(z,1)
	GP_ey_check[i] 	 = gpFullKin(z[i,:],u[i,:],feature_GP,prep_ey)
	GP_epsi_check[i] = gpFullKin(z[i,:],u[i,:],feature_GP,prep_epsi)
end

figure("GPR play back: $(file_name)")
subplot(3,2,1); plot(erro_s_his,     "-",alpha=0.5) 
subplot(3,2,2); plot(erro_ey_his,    "-",alpha=0.5); plot(GP_ey_his[:,1],  "o",alpha=0.3); plot(GP_ey_check[:,1],  "o",alpha=0.6)
subplot(3,2,3); plot(erro_epsi_his,  "-",alpha=0.5); plot(GP_epsi_his[:,1],"o",alpha=0.3); plot(GP_epsi_check[:,1],"o",alpha=0.6)
subplot(3,2,4); plot(erro_vx_his,    "-",alpha=0.5)
subplot(3,2,5); plot(erro_vy_his,    "-",alpha=0.5)
subplot(3,2,6); plot(erro_psiDot_his,"-",alpha=0.5)
show()