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

    track = create_track(0.2)
    hold(1)
    plot(z.z[:,1],z.z[:,2],"-",gps_meas.z[:,1]/100,gps_meas.z[:,2]/100,".",est.z[:,1],est.z[:,2],"-")
    plot(track[:,1],track[:,2],"b-",track[:,3],track[:,4],"r-",track[:,5],track[:,6],"r-")
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
    plot(oldTraj[:,1,1,1],oldTraj[:,2:4,1,1],"-o")
    legend(["e_y","e_psi","v"])
    grid(1)
    figure()
    ax1=subplot(211)
    plot(t,state)
    legend(["s","e_y","e_psi","v"])
    grid(1)
    #figure()
    subplot(212,sharex = ax1)
    plot(t,cost)
    grid(1)
    legend(["costZ","costZTerm","constZTerm","derivCost","controlCost","laneCost"])
    figure()
    plot(t,curv)
    legend(["1","2","3","4","5","6","7","8","9"])
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
    ss = [s.^8 s.^7 s.^6 s.^5 s.^4 s.^3 s.^2 s.^1 s.^0]
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

function create_track(w)
    x = [0.0]           # starting point
    y = [0.0]
    x_l = [0.0]           # starting point
    y_l = [w]
    x_r = [0.0]           # starting point
    y_r = [-w]
    ds = 0.06

    theta = [0.0]

    add_curve(theta,10,0.0)
    add_curve(theta,50,-2*pi/3)
    add_curve(theta,70,pi)
    add_curve(theta,60,-5*pi/6)
    add_curve(theta,10,0.0)
    add_curve(theta,30,-pi/2)
    add_curve(theta,40,0.0)
    add_curve(theta,20,-pi/4)
    add_curve(theta,20,pi/4)
    add_curve(theta,50,-pi/2)
    add_curve(theta,22,0.0)
    add_curve(theta,30,-pi/2)
    add_curve(theta,14,0.0)

    for i=1:length(theta)
            push!(x, x[end] + cos(theta[i])*ds)
            push!(y, y[end] + sin(theta[i])*ds)
            push!(x_l, x[end-1] + cos(theta[i]+pi/2)*w)
            push!(y_l, y[end-1] + sin(theta[i]+pi/2)*w)
            push!(x_r, x[end-1] + cos(theta[i]-pi/2)*w)
            push!(y_r, y[end-1] + sin(theta[i]-pi/2)*w)
    end
    track = cat(2, x, y, x_l, y_l, x_r, y_r)
    return track
    #plot(x,y,x_l,y_l,x_r,y_r)
end

function add_curve(theta::Array{Float64}, length::Int64, angle)
    d_theta = 0
    curve = 2*sum(1:length/2)+length/2
    for i=0:length-1
        if i < length/2+1
            d_theta = d_theta + angle / curve
        else
            d_theta = d_theta - angle / curve
        end
        push!(theta, theta[end] + d_theta)
    end
end
