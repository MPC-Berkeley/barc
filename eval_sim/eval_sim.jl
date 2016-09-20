using JLD
using PyPlot
using HDF5, JLD, ProfileView

type Measurements{T}
    i::Int64          # measurement counter
    t::Array{T}       # time data
    z::Array{T}       # measurement values
end


const log_path          = "$(homedir())/simulations/output.jld"
const log_path_LMPC     = "$(homedir())/simulations/output_LMPC.jld"
const log_path_profile  = "$(homedir())/simulations/profile.jlprof"

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
    t       = d["t"]
    state   = d["state"]
    sol_z   = d["sol_z"]
    sol_u   = d["sol_u"]
    cost    = d["cost"]
    curv    = d["curv"]
    plot(oldTraj[:,:,1,1])
    grid(1)
    figure()
    plot(t,state)
    grid(1)
    plot(t,cost)
    grid(1)
    legend(["costZ","costZTerm","constZTerm","derivCost","controlCost","laneCost"])
    figure()
    plot(t,curv)
    legend(["1","2","3","4","5","6","7"])
end

function anim_MPC(z)
    figure()
    hold(0)
    grid(1)
    for i=1:size(z,3)
        plot(z[:,:,i])
    xlim([1,11])
    ylim([-2,2])
        sleep(0.1)
    end
end

function anim_curv(curv)
    s = 0.0:.05:2.0
    figure()
    hold(0)
    ss = [s.^6 s.^5 s.^4 s.^3 s.^2 s.^1 s.^0]
    for i=1:size(curv,1)
        c = ss*curv[i,:]'
        plot(s,c)
        xlim([0,2])
        ylim([-1.5,1.5])
        sleep(0.25)
    end
end



function eval_prof()
    Profile.clear()
    @load "$(homedir())/simulations/profile.jlprof"
    ProfileView.view(li, lidict=lidict)
end