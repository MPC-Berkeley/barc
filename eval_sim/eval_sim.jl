using JLD
using PyPlot
using HDF5, JLD, ProfileView

type Measurements{T}
    i::Int64          # measurement counter
    t::Array{Float64}       # time data
    z::Array{T}       # measurement values
end


const log_path          = "$(homedir())/simulations/output.jld"
const log_path_LMPC     = "$(homedir())/simulations/output_LMPC.jld"
const log_path_profile  = "$(homedir())/simulations/profile.jlprof"


function simModel(z,u,dt,l_A,l_B)

   # kinematic bicycle model
   # u[1] = acceleration
   # u[2] = steering angle

    bta = atan(l_A/(l_A+l_B)*tan(u[2]))

    zNext = z
    zNext[1] = z[1] + dt*(z[4]*cos(z[3] + bta))       # x
    zNext[2] = z[2] + dt*(z[4]*sin(z[3] + bta))     # y
    zNext[3] = z[3] + dt*(z[4]/l_B*sin(bta))        # psi
    zNext[4] = z[4] + dt*(u[1] - 0.63 * z[4]^2 * sign(z[4]))                     # v

    return zNext
end

function eval_sim()
    d = load(log_path)

    est         = d["estimate"]
    est_dyn     = d["estimate_dyn"]
    imu_meas    = d["imu_meas"]
    gps_meas    = d["gps_meas"]
    z           = d["z"]
    cmd_log     = d["cmd_log"]
    slip_a      = d["slip_a"]

    t0 = est.t[1]
    track = create_track(0.3)

    figure()
    ax1=subplot(311)
    plot(z.t-t0,z.z,"-o")
    grid()
    legend(["x","y","v_x","v_y","psi","psi_dot","d_f"])
    subplot(312,sharex=ax1)
    plot(cmd_log.t-t0,cmd_log.z,"-o")
    grid()
    legend(["u","d_f"])
    subplot(313,sharex=ax1)
    plot(slip_a.t-t0,slip_a.z,"-o")
    grid()
    legend(["a_f","a_r"])

    figure()
    plot(z.z[:,1],z.z[:,2],"-",gps_meas.z[:,1]/100,gps_meas.z[:,2]/100,".",est.z[:,1],est.z[:,2],"-")
    plot(track[:,1],track[:,2],"b.",track[:,3],track[:,4],"r-",track[:,5],track[:,6],"r-")
    grid(1)
    title("x-y-view")
    axis("equal")
    legend(["Real state","GPS meas","estimate"])
    
    figure()
    plot(est_dyn.t,est_dyn.z,"-*",est.t,est.z,"--",z.t,z.z,"-")
    title("Dyn. est. -*, est. --, real -")
    grid()
    legend(["x","y","v_x","v_y","psi","psi_dot","x","y","psi","v","x","y","v_x","v_y","psi","psi_dot","d_f"])

    figure()
    title("Comparison of psi")
    plot(imu_meas.t,imu_meas.z,"-x",z.t,z.z[:,5:6],est_dyn.t,est_dyn.z[:,5:6],"-*")
    legend(["imu_psi","imu_psi_dot","real_psi","real_psi_dot","est_psi","est_psi_dot"])
    grid()

    figure()
    title("Comparison of v")
    plot(z.t,z.z[:,3:4],est_dyn.t,est_dyn.z[:,3:4],"-*")
    legend(["real_xDot","real_yDot","est_xDot","est_yDot"])
    grid()

    figure()
    title("Comparison of x,y")
    plot(z.t,z.z[:,1:2],est_dyn.t,est_dyn.z[:,1:2],"-*")
    legend(["real_x","real_y","est_x","est_y"])
    grid()

    #figure()
    #plot(est.z[:,1],est.z[:,2],"x",est_dyn.z[:,1],est_dyn.z[:,2],"-*",z.z[:,1],z.z[:,2],"-")
    #grid(1)
    #legend(["est","est_dyn","true"])

    #figure()
    #plot(z.t-t0,z.z[:,5],imu_meas.t-t0,imu_meas.z,est.t-t0,est.z[:,3])
    #grid(1)
    #legend(["Real psi","psi meas","estimate"])

    figure()
    plot(cmd_log.t-t0,cmd_log.z)
    legend(["a","d_f"])
    grid()
end

function eval_LMPC()
    d_sim       = load(log_path)
    d_lmpc      = load(log_path_LMPC)

    oldTraj     = d_lmpc["oldTraj"]
    t           = d_lmpc["t"]
    state       = d_lmpc["state"]
    sol_z       = d_lmpc["sol_z"]
    sol_u       = d_lmpc["sol_u"]
    cost        = d_lmpc["cost"]
    curv        = d_lmpc["curv"]

    x_est       = d_lmpc["x_est"]
    coeffX      = d_lmpc["coeffX"]
    coeffY      = d_lmpc["coeffY"]
    s_start     = d_lmpc["s_start"]
    est         = d_sim["estimate"]
    imu_meas    = d_sim["imu_meas"]
    gps_meas    = d_sim["gps_meas"]
    z           = d_sim["z"]
    cmd_log     = d_sim["cmd_log"]

    t0 = t[1]

    figure()
    c = zeros(size(curv,1),1)
    for i=1:size(curv,1)
        s = state[i,1]
        c[i] = ([s.^8 s.^7 s.^6 s.^5 s.^4 s.^3 s.^2 s.^1 s.^0] * curv[i,:]')[1]
    end
    plot(s_start+state[:,1],c,"-o")
    for i=1:2:size(curv,1)
        s = sol_z[:,1,i]
        c = zeros(size(curv,1),1)
        c = [s.^8 s.^7 s.^6 s.^5 s.^4 s.^3 s.^2 s.^1 s.^0] * curv[i,:]'
        plot(s_start[i]+s,c,"-*")
    end
    title("Curvature over path")
    xlabel("Curvilinear abscissa [m]")
    ylabel("Curvature")
    grid()

    track = create_track(0.3)
    figure()
    hold(1)
    plot(x_est[:,1],x_est[:,2],"-o")
    legend(["Estimated position"])
    plot(track[:,1],track[:,2],"b.",track[:,3],track[:,4],"r-",track[:,5],track[:,6],"r-")
    axis("equal")
    grid(1)
    # for i=1:size(x_est,1)
    #     #dir = [cos(x_est[i,3]) sin(x_est[i,3])]
    #     #dir2 = [cos(x_est[i,3] - state[i,3]) sin(x_est[i,3] - state[i,3])]
    #     #lin = [x_est[i,1:2];x_est[i,1:2] + 0.05*dir]
    #     #lin2 = [x_est[i,1:2];x_est[i,1:2] + 0.05*dir2]
    #     #plot(lin[:,1],lin[:,2],"-o",lin2[:,1],lin2[:,2],"-*")
    # end
    for i=1:4:size(x_est,1)
            z_pred = zeros(11,4)
            z_pred[1,:] = x_est[i,:]
            for j=2:11
                z_pred[j,:] = simModel(z_pred[j-1,:],sol_u[j-1,:,i],0.1,0.125,0.125)
            end
            plot(z_pred[:,1],z_pred[:,2],"-*")
    end

    # for i=1:size(x_est,1)
    #     s = 0.4:.1:2.5
    #     ss = [s.^6 s.^5 s.^4 s.^3 s.^2 s.^1 s.^0]
    #     x = ss*coeffX[i,:]'
    #     y = ss*coeffY[i,:]'
    #     plot(x,y)
    # end

    # rg = 100:500
    # figure()
    # plot(s_start[rg]+state[rg,1],state[rg,2:4],"-o")
    # title("Comparison states and prediction")
    # legend(["ey","epsi","v"])
    # grid(1)
    # for i=100:5:500
    #     plot(s_start[i]+sol_z[:,1,i],sol_z[:,2:4,i],"-*")
    # end
    figure()
    plot(oldTraj[:,1,1,1],oldTraj[:,2:4,1,1],"-o")
    legend(["e_y","e_psi","v"])
    grid(1)

    figure()
    plot(s_start+state[:,1],state[:,[2,4]],"*")
    grid()

    figure()
    ax1=subplot(211)
    plot(t-t0,state,z.t-t0,z.z[:,1:2])
    legend(["s","e_y","e_psi","v","x","y"])
    grid(1)
    subplot(212,sharex = ax1)
    plot(t-t0,cost)
    grid(1)
    legend(["costZ","costZTerm","constZTerm","derivCost","controlCost","laneCost"])
    # figure()
    # plot(1:size(curv,1),curv)
    # grid()
    # title("Polynomial coefficients")
    # legend(["1","2","3","4","5","6","7","8","9"])
end

function eval_oldTraj(i)
    d = load(log_path_LMPC)
    oldTraj = d["oldTraj"]
    plot(oldTraj[:,1,1,i],oldTraj[:,2:4,1,i],"-o",oldTraj[:,1,2,i],oldTraj[:,2:4,2,i],"-*")
end

function eval_LMPC_coeff(k)
    d           = load(log_path_LMPC)
    oldTraj     = d["oldTraj"]
    sol_z       = d["sol_z"]
    sol_u       = d["sol_u"]
    coeffCost   = d["coeffCost"]
    coeffConst  = d["coeffConst"]
    s_start     = d["s_start"]

    s   = sol_z[:,1,k]
    ss  = [s.^5 s.^4 s.^3 s.^2 s.^1 s.^0]
    subplot(311)
    plot(s,sol_z[:,2,k],"-o",s,ss*coeffConst[:,1,1,k],s,ss*coeffConst[:,2,1,k])
    grid()
    title("Position = $(s_start[k] + s[1]), k = $k")
    xlabel("s")
    ylabel("e_Y")
    subplot(312)
    plot(s,sol_z[:,3,k],"-o",s,ss*coeffConst[:,1,2,k],s,ss*coeffConst[:,2,2,k])
    grid()
    xlabel("s")
    ylabel("e_Psi")
    subplot(313)
    plot(s,sol_z[:,4,k],"-o",s,ss*coeffConst[:,1,3,k],s,ss*coeffConst[:,2,3,k])
    grid()
    xlabel("s")
    ylabel("v")
end

function anim_LMPC(k1,k2)
    d           = load(log_path_LMPC)
    oldTraj     = d["oldTraj"]
    sol_z       = d["sol_z"]
    sol_u       = d["sol_u"]
    coeffCost   = d["coeffCost"]
    coeffConst  = d["coeffConst"]
    s_start     = d["s_start"]
    state       = d["state"]

    N = size(sol_z,1)-1

    for k=k1:k2
        s = sol_z[1:10,1,k]
        subplot(211)
        plot(s,sol_u[:,1,k],"-o")
        ylim([-1,2])
        xlim([0,2])
        grid()
        subplot(212)
        plot(s,sol_u[:,2,k],"-o")
        ylim([-0.5,0.5])
        xlim([0,2])
        grid()
        sleep(0.1)
    end

    figure()
    hold(0)
    for k=k1:k2
        s_state = s_start[k:k+N] + state[k:k+N,1] - s_start[k]
        s   = sol_z[:,1,k]
        ss  = [s.^5 s.^4 s.^3 s.^2 s.^1 s.^0]
        subplot(311)
        plot(s,sol_z[:,2,k],"-o",s_state,state[k:k+N,2],"-*",s,ss*coeffConst[:,1,1,k],s,ss*coeffConst[:,2,1,k])
        grid()
        title("Position = $(s_start[k] + s[1]), k = $k")
        xlabel("s")
        xlim([0,3])
        ylim([-0.2,0.2])
        ylabel("e_Y")
        subplot(312)
        plot(s,sol_z[:,3,k],"-o",s_state,state[k:k+N,3],"-*",s,ss*coeffConst[:,1,2,k],s,ss*coeffConst[:,2,2,k])
        grid()
        xlabel("s")
        ylabel("e_Psi")
        xlim([0,3])
        ylim([-0.2,0.2])
        subplot(313)
        plot(s,sol_z[:,4,k],"-o",s_state,state[k:k+N,4],"-*",s,ss*coeffConst[:,1,3,k],s,ss*coeffConst[:,2,3,k])
        grid()
        xlabel("s")
        ylabel("v")
        xlim([0,3])
        ylim([0,2])
        sleep(0.1)
    end
end

function anim_MPC(z)
    figure()
    hold(0)
    grid(1)
    for i=1:size(z,3)
        plot(z[:,:,i])
    xlim([1,11])
    ylim([-2,2])
        sleep(0.01)
    end
end

function anim_curv(curv)
    s = 0.0:.05:2.0
    figure()
    hold(0)
    #ss = [s.^10 s.^9 s.^8 s.^7 s.^6 s.^5 s.^4 s.^3 s.^2 s.^1 s.^0]
    ss = [s.^6 s.^5 s.^4 s.^3 s.^2 s.^1 s.^0]
    for i=1:size(curv,1)
        c = ss*curv[i,:]'
        plot(s,c)
        xlim([0,2])
        ylim([-1.5,1.5])
        sleep(0.1)
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

    # SOPHISTICATED TRACK
    # add_curve(theta,30,0.0)
    # add_curve(theta,60,-2*pi/3)
    # add_curve(theta,90,pi)
    # add_curve(theta,80,-5*pi/6)
    # add_curve(theta,10,0.0)
    # add_curve(theta,50,-pi/2)
    # add_curve(theta,50,0.0)
    # add_curve(theta,40,-pi/4)
    # add_curve(theta,30,pi/4)
    # add_curve(theta,20,0.0)
    # add_curve(theta,50,-pi/2)
    # add_curve(theta,25,0.0)
    # add_curve(theta,50,-pi/2)
    # add_curve(theta,28,0.0)

    # SIMPLE track
    add_curve(theta,50,0)
    add_curve(theta,100,-pi)
    add_curve(theta,100,0)
    add_curve(theta,100,-pi)
    add_curve(theta,49,0)

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
