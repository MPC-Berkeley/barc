using JLD
using PyPlot
using HDF5, JLD, ProfileView

type Measurements{T}
    i::Int64          # measurement counter
    t::Array{T}       # time data
    z::Array{T}       # measurement values
end


log_path = "$(homedir())/simulations/output.jld"
log_path_LMPC = "$(homedir())/simulations/LMPC_output.jld"
log_path_profile = "$(homedir())/simulations/profile.jlprof"

function eval_sim()
    d = load(log_path)

    est         = d["estimate"]
    imu_meas    = d["imu_meas"]
    gps_meas    = d["gps_meas"]
    z           = d["z"]
    cmd_log     = d["cmd_log"]

    plot(z.z[:,1],z.z[:,2],"-",gps_meas.z[:,1]/100,gps_meas.z[:,2]/100,".",est.z[:,1],est.z[:,2],"-")
    grid(1)
    legend(["real state","GPS meas","estimate"])
    figure()
    plot(z.t,z.z[:,3],imu_meas.t,imu_meas.z,est.t,est.z[:,3])
    grid(1)
    legend(["Real psi","psi meas","estimate"])
    figure()
    plot(z.t,z.z[:,4])
    grid()
    legend(["Velocity"])
    figure()
    plot(cmd_log.t,cmd_log.z)
    legend(["a","d_f"])
    grid()
end

function eval_LMPC()
    d = load(log_path_LMPC)
    oldTraj = d["oldTraj"]
    plot(oldTraj[:,:,1,1])
    grid(1)
end

function eval_prof()
    Profile.clear()
    @load "$(homedir())/simulations/profile.jlprof"
    ProfileView.view(li, lidict=lidict)
end