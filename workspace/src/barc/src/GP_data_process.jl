# THIS FILE NEEDS TO BE RUN AFTER THE FEATURE DATA COLLECTING TO GET THE GP DATA
using JLD
using PyPlot
include("barc_lib/classes.jl")
include("barc_lib/LMPC/functions.jl")

data = load("$(homedir())/simulations/Feature_Data/FeatureDataCollecting.jld")
z = data["feature_z"]
u = data["feature_u"]

mpcCoeff        = MpcCoeff()
mpcCoeff.c_Vx   = zeros(1,3)
mpcCoeff.c_Vy   = zeros(1,4)
mpcCoeff.c_Psi  = zeros(1,3)

modelParams     = ModelParams()
modelParams.l_A = 0.125
modelParams.l_B = 0.125
modelParams.dt  = 0.1
modelParams.m   = 1.98
modelParams.I_z = 0.03
modelParams.c_f = 0.05

v = 2.5
max_a=7.6;
R=v^2/max_a
max_c=1/R
angle=(pi+pi/2)-0.105
R_kin = 0.8
num_kin = Int(round(angle/ ( 0.03/R_kin ) * 2))
num = max(Int(round(angle/ ( 0.03/R ) * 2)),num_kin)
track_data=[num -angle;
            num  angle]
track = Track(track_data)

feature_GP_vy_e     = zeros(size(z,1))
feature_GP_psidot_e = zeros(size(z,1))
for i=1:size(z,1)
    (iden_z,iden_u,~)=find_feature_dist(z,u,z[i,:,1],u[i,:])
    (mpcCoeff.c_Vx,mpcCoeff.c_Vy,mpcCoeff.c_Psi)=coeff_iden_dist(iden_z,iden_u)
    z_next=car_sim_iden_tv(z[i,:,1],u[i,:],0.1,mpcCoeff,modelParams,track)
    feature_GP_vy_e[i] = z[i,5,2] - z_next[5]
    feature_GP_psidot_e[i] = z[i,6,2] - z_next[6]
end

feature_GP_z = z[:,:,1]
feature_GP_u = u

log_path = "$(homedir())/simulations/Feature_Data/FeatureData_GP.jld"
save(log_path,"feature_GP_vy_e",feature_GP_vy_e,"feature_GP_psidot_e",feature_GP_psidot_e,"feature_GP_z",feature_GP_z,"feature_GP_u",feature_GP_u)


figure()
# plot(z[:,5,2]-z[:,5,1])
plot(feature_GP_vy_e)

figure()
plot(z[:,6,2]-z[:,6,1])
plot(feature_GP_psidot_e)