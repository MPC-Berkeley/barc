using JLD
using PyPlot
using PyCall
@pyimport matplotlib.animation as animation
using HDF5, JLD, ProfileView
# pos_info[1]  = s
# pos_info[2]  = eY
# pos_info[3]  = ePsi
# pos_info[4]  = v
# pos_info[5]  = s_start
# pos_info[6]  = x
# pos_info[7]  = y
# pos_info[8]  = v_x
# pos_info[9]  = v_y
# pos_info[10] = psi
# pos_info[11] = psiDot
# pos_info[12] = x_raw
# pos_info[13] = y_raw
# pos_info[14] = psi_raw
# pos_info[15] = v_raw
# pos_info[16] = psi_drift
# pos_info[17] = a_x
# pos_info[18] = a_y

include("../workspace/src/barc/src/barc_lib/classes.jl")

type Measurements{T}
    i::Int64                # measurement counter
    t::Array{Float64}       # time data (when it was received by this recorder)
    t_msg::Array{Float64}   # time that the message was sent
    z::Array{T}             # measurement values
end

# THIS FUNCTION EVALUATES DATA THAT WAS LOGGED BY THE SIMULATOR (INCLUDES "REAL" SIMULATION DATA)
# ***********************************************************************************************

function eval_sim(code::AbstractString)
    log_path_sim = "$(homedir())/simulations/output-SIM-$(code).jld"
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    d_sim = load(log_path_sim)
    d_rec = load(log_path_record)

    imu_meas    = d_sim["imu_meas"]
    gps_meas    = d_sim["gps_meas"]
    z           = d_sim["z"]
    cmd_log     = d_sim["cmd_log"]
    slip_a      = d_sim["slip_a"]
    pos_info    = d_rec["pos_info"]
    vel_est     = d_rec["vel_est"]

    t0 = pos_info.t[1]
    track = create_track(0.4)

    figure()
    ax1=subplot(311)
    plot(z.t-t0,z.z,"-*")
    title("Real states")
    grid()
    legend(["x","y","v_x","v_y","psi","psi_dot","a","d_f"])
    subplot(312,sharex=ax1)
    plot(cmd_log.t-t0,cmd_log.z,"-*")
    title("Inputs")
    grid()
    legend(["u","d_f"])
    subplot(313,sharex=ax1)
    plot(slip_a.t-t0,slip_a.z,"-*")
    title("Slip angles")
    grid()
    legend(["a_f","a_r"])

    figure()
    plot(z.z[:,1],z.z[:,2],"-",gps_meas.z[:,1],gps_meas.z[:,2],".",pos_info.z[:,6],pos_info.z[:,7],"-")
    plot(track[:,1],track[:,2],"b.",track[:,3],track[:,4],"r-",track[:,5],track[:,6],"r-")
    grid(1)
    title("x-y-view")
    axis("equal")
    legend(["Real state","GPS meas","Estimate"])
    
    figure()
    title("Comparison of psi")
    plot(imu_meas.t-t0,imu_meas.z,"-x",z.t-t0,z.z[:,5:6],pos_info.t-t0,pos_info.z[:,10:11],"-*")
    legend(["imu_psi","imu_psi_dot","real_psi","real_psi_dot","est_psi","est_psi_dot"])
    grid()

    figure()
    title("Comparison of v")
    plot(z.t-t0,z.z[:,3:4],z.t-t0,sqrt(z.z[:,3].^2+z.z[:,4].^2),pos_info.t-t0,pos_info.z[:,8:9],"-*",pos_info.t-t0,sqrt(pos_info.z[:,8].^2+pos_info.z[:,9].^2),"-*",vel_est.t-t0,vel_est.z)
    legend(["real_xDot","real_yDot","real_v","est_xDot","est_yDot","est_v","v_x_meas"])
    grid()

    figure()
    title("Comparison of x,y")
    plot(z.t-t0,z.z[:,1:2],pos_info.t-t0,pos_info.z[:,6:7],"-*",gps_meas.t-t0,gps_meas.z)
    legend(["real_x","real_y","est_x","est_y","meas_x","meas_x"])
    grid()
end

# THIS FUNCTION EVALUATES DATA THAT WAS RECORDED BY BARC_RECORD.JL
# ****************************************************************
function eval_run(code::AbstractString)
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    #log_path_record = "$(homedir())/open_loop/output-record-0ed5.jld"
    d_rec = load(log_path_record)

    imu_meas    = d_rec["imu_meas"]
    gps_meas    = d_rec["gps_meas"]
    cmd_log     = d_rec["cmd_log"]
    cmd_pwm_log = d_rec["cmd_pwm_log"]
    vel_est     = d_rec["vel_est"]
    pos_info    = d_rec["pos_info"]

    t0      = pos_info.t[1]
    track   = create_track(0.4)

    # Calculate accelerations
    acc = smooth(diff(smooth(vel_est.z[:,1],10))./diff(vel_est.t),10)
    figure(10)
    plot(vel_est.t[1:end-1]-t0,acc,cmd_log.t-t0,cmd_log.z[:,1])
    grid("on")
    legend(["Acc","u_a"])


    figure()
    plot(gps_meas.z[:,1],gps_meas.z[:,2],"-.",pos_info.z[:,6],pos_info.z[:,7],"-*")
    plot(track[:,1],track[:,2],"b.",track[:,3],track[:,4],"r-",track[:,5],track[:,6],"r-")
    grid(1)
    title("x-y-view")
    axis("equal")
    legend(["GPS meas","estimate"])
    
    figure()
    plot(imu_meas.t-t0,imu_meas.z[:,7:9])
    plot(pos_info.t-t0,pos_info.z[:,19:20])
    grid("on")
    title("Measured accelerations")

    figure()
    plot(gps_meas.t-t0,gps_meas.z,"-*",pos_info.t-t0,pos_info.z[:,6:7],"-x",gps_meas.t_msg-t0,gps_meas.z,"--x")
    grid(1)
    title("GPS comparison")
    xlabel("t [s]")
    ylabel("Position [m]")
    legend(["x_meas","y_meas","x_est","y_est"])

    figure()
    ax2=subplot(211)
    title("Commands")
    plot(cmd_log.t-t0,cmd_log.z,"-*",cmd_log.t_msg-t0,cmd_log.z,"-x")
    grid("on")
    xlabel("t [s]")
    subplot(212,sharex=ax2)
    plot(cmd_pwm_log.t-t0,cmd_pwm_log.z,"-*")
    grid("on")
    xlabel("t [s]")

    figure()
    title("Comparison of psi")
    plot(imu_meas.t-t0,imu_meas.z[:,6],imu_meas.t-t0,imu_meas.z[:,3],"-x",pos_info.t-t0,pos_info.z[:,10:11],"-*",pos_info.t-t0,pos_info.z[:,16],"-*",pos_info.t-t0,pos_info.z[:,14])
    legend(["imu_psi","imu_psi_dot","est_psi","est_psi_dot","psi_drift","psi_raw"])
    grid()

    figure()
    plot(pos_info.t-t0,pos_info.z[:,2:3])
    legend(["e_y","e_psi"])
    title("Deviations from reference")
    grid("on")
    xlabel("t [s]")
    ylabel("eY [m], ePsi [rad]")

    # figure()
    # title("Raw IMU orientation data")
    # plot(imu_meas.t-t0,imu_meas.z[:,1:3],"--",imu_meas.t-t0,imu_meas.z[:,4:6],imu_meas.t_msg-t0,imu_meas.z[:,4:6])
    # grid("on")
    # legend(["w_x","w_y","w_z","roll","pitch","yaw"])

    figure()
    title("v measurements and estimate")
    plot(vel_est.t-t0,vel_est.z[:,1],"-x",pos_info.t-t0,pos_info.z[:,[8:9,4]],"-+")
    legend(["v_raw","est_xDot","est_yDot","est_v"])
    grid()

    # figure()
    # title("s, eY and inputs")
    # ax1=subplot(211)
    # plot(pos_info.t-t0,pos_info.z[:,2:3],"-*")
    # grid("on")
    # legend(["eY","ePsi"])
    # subplot(212,sharex=ax1)
    # plot(cmd_log.t-t0,cmd_log.z,"-*")
    # grid("on")
    # legend(["u","d_f"])
    nothing
end

function plot_v_ey_over_s(code::AbstractString,laps::Array{Int64})
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    d_lmpc      = load(log_path_LMPC)

    oldTraj     = d_lmpc["oldTraj"]
    n_laps = size(laps,1)

    println("OldCost: ",oldTraj.oldCost)
    # plot v_x over s   
    figure(1)
    for i=1:n_laps
        idx = (oldTraj.oldTraj[:,6,laps[i]] .>= 0.0) & (oldTraj.oldTraj[:,6,laps[i]] .<= 19.11)
        plot(oldTraj.oldTraj[idx,6,laps[i]],oldTraj.oldTraj[idx,1,laps[i]],label="Lap $(laps[i])")
    end
    legend()
    xlabel("s [m]")
    ylabel("v [m/s]")

    # plot e_y over s   
    figure(2)
    for i=1:n_laps
        idx = (oldTraj.oldTraj[:,6,laps[i]] .>= 0.0) & (oldTraj.oldTraj[:,6,laps[i]] .<= 19.11)
        plot(oldTraj.oldTraj[idx,6,laps[i]],oldTraj.oldTraj[idx,5,laps[i]],label="Lap $(laps[i])")
    end
    legend()
    xlabel("s [m]")
    ylabel("e_Y [m]")
end

function plot_v_over_xy(code::AbstractString,lap::Int64)
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    d_rec       = load(log_path_record)
    d_lmpc      = load(log_path_LMPC)

    oldTraj     = d_lmpc["oldTraj"]
    pos_info    = d_rec["pos_info"]

    # Find timing of selected lap:
    t_start = oldTraj.oldTimes[oldTraj.oldTraj[:,6,lap].>=0,lap][1]
    t_end = oldTraj.oldTimes[oldTraj.oldTraj[:,6,lap].<=19.11,lap][end]
    println("Laptime = $(t_end-t_start) s")

    idx = (pos_info.t.>=t_start) & (pos_info.t.<=t_end)

    track   = create_track(0.4)

    figure()
    plot(track[:,1],track[:,2],"b.",track[:,3],track[:,4],"r-",track[:,5],track[:,6],"r-")
    scatter(pos_info.z[idx,6],pos_info.z[idx,7],c=pos_info.z[idx,8],cmap=ColorMap("jet"),edgecolors="face")
    grid(1)
    title("x-y-view")
    axis("equal")
    cb = colorbar()
    cb[:set_label]("Velocity [m/s]")
    println("Average v_x = ",mean(pos_info.z[idx,8])," m/s")

end


function eval_open_loop(code::AbstractString)
    log_path_record = "$(homedir())/open_loop/output-record-$(code).jld"
    d_rec = load(log_path_record)

    imu_meas    = d_rec["imu_meas"]
    gps_meas    = d_rec["gps_meas"]
    cmd_log     = d_rec["cmd_log"]
    cmd_pwm_log = d_rec["cmd_pwm_log"]
    vel_est     = d_rec["vel_est"]
    pos_info    = d_rec["pos_info"]

    t0      = pos_info.t[1]

    figure()
    ax3=subplot(211)
    title("Comparison speed and input")
    plot(vel_est.t-t0,vel_est.z,"-*",vel_est.t_msg-t0,vel_est.z,"-x")
    legend(["vel_est"])
    grid("on")
    subplot(212,sharex=ax3)
    plot(cmd_pwm_log.t-t0,cmd_pwm_log.z[:,1],cmd_pwm_log.t_msg-t0,cmd_pwm_log.z[:,1])
    grid("on")

    gps_speed_raw = diff(gps_meas.z)./diff(gps_meas.t)
    gps_speed = [0;sqrt(gps_speed_raw[:,1].^2+gps_speed_raw[:,2].^2)]
    figure()
    title("Comparison GPS and encoder speed")
    plot(vel_est.t-t0,vel_est.z,"-*",gps_meas.t-t0,gps_speed,"-x",pos_info.t_msg-t0,pos_info.z[:,4],"-+")
    grid("on")
    legend(["encoder","gps","estimator"])

    figure()
    title("Acceleration data")
    plot(imu_meas.t-t0,imu_meas.z[:,7:9],"-x",imu_meas.t_msg-t0,imu_meas.z[:,7:9],"-*")
    legend(["a_x","a_y","a_z"])
    grid()

    figure()
    plot(gps_meas.t-t0,gps_meas.z,"-*",pos_info.t-t0,pos_info.z[:,12:13],"-x",pos_info.t-t0,pos_info.z[:,6:7])
    legend(["x_gps","y_gps","x_filtered","y_filtered","x_est","y_est"])
    grid("on")

    figure()
    plot(gps_meas.t-t0,gps_meas.z,"-*")
    legend(["x_gps","y_gps"])
    xlabel("t [s]")
    ylabel("Position [m]")
    grid("on")
end



# THIS FUNCTION EVALUATES MPC-SPECIFIC DATA
# *****************************************
function eval_LMPC(code::AbstractString)
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    d_rec       = load(log_path_record)
    d_lmpc      = load(log_path_LMPC)

    oldTraj     = d_lmpc["oldTraj"]
    t           = d_lmpc["t"]
    state       = d_lmpc["state"]
    sol_z       = d_lmpc["sol_z"]
    sol_u       = d_lmpc["sol_u"]
    cost        = d_lmpc["cost"]
    curv        = d_lmpc["curv"]
    c_Vx        = d_lmpc["c_Vx"]
    c_Vy        = d_lmpc["c_Vy"]
    c_Psi       = d_lmpc["c_Psi"]
    cmd         = d_lmpc["cmd"]                 # this is the command how it was sent by the MPC
    step_diff   = d_lmpc["step_diff"]           # this is the command how it was sent by the MPC

    x_est       = d_lmpc["x_est"]
    coeffX      = d_lmpc["coeffX"]
    coeffY      = d_lmpc["coeffY"]
    imu_meas    = d_rec["imu_meas"]
    gps_meas    = d_rec["gps_meas"]
    cmd_log     = d_rec["cmd_log"]              # this is the command how it was received by the simulator
    pos_info    = d_rec["pos_info"]

    t0 = t[1]

    figure(2)
    ax1=subplot(311)
    plot(pos_info.t-t0,pos_info.z[:,8],".",t-t0,state[:,1],"-*")
    title("Comparison of publishing and receiving time")
    legend(["x_dot_estimate","x_dot_MPC"])
    grid("on")
    xlabel("t [s]")
    ylabel("v_x [m/s]")
    subplot(312,sharex=ax1)
    plot(cmd_log.t-t0,cmd_log.z,"-o",t-t0,cmd[1:length(t),:],"-*",t-t0,state[:,7])
    legend(["a_rec","d_f_rec","a_MPC","d_f_MPC","acc_filter"])
    grid("on")
    subplot(313,sharex=ax1)
    plot(t-t0,c_Vx)
    title("System ID xDot coefficients")
    grid("on")
    legend(["1","2","3"])

    figure(3)
    plot(t,state)
    grid("on")
    legend(["v_x","v_y","psiDot","ePsi","eY","s"])

    figure()
    subplot(311)
    title("c_Vx")
    plot(t-t0,c_Vx)
    grid("on")
    subplot(312)
    title("c_Vy")
    plot(t-t0,c_Vy)
    grid("on")
    subplot(313)
    title("c_Psi")
    plot(t-t0,c_Psi)
    grid("on")

    # *********** CURVATURE *********************
    figure()
    c = zeros(size(curv,1),1)
    for i=1:size(curv,1)
        s = state[i,6]
        #c[i] = ([s.^8 s.^7 s.^6 s.^5 s.^4 s.^3 s.^2 s.^1 s.^0] * curv[i,:]')[1]
        c[i] = ([s.^3 s.^2 s.^1 s.^0] * curv[i,:]')[1]
    end
    plot(state[:,6],c,"-o")
    for i=1:5:size(curv,1)
        if isnan(sol_z[1,6,i])
            s = sol_z[:,1,i]
        else
            s = sol_z[:,6,i]
        end
        c = zeros(size(curv,1),1)
        #c = [s.^8 s.^7 s.^6 s.^5 s.^4 s.^3 s.^2 s.^1 s.^0] * curv[i,:]'
        c = [s.^3 s.^2 s.^1 s.^0] * curv[i,:]'
        plot(s,c,"-*")
    end
    title("Curvature over path")
    xlabel("Curvilinear abscissa [m]")
    ylabel("Curvature")
    grid()

    track = create_track(0.4)
    figure()
    hold(1)
    plot(x_est[:,1],x_est[:,2],"-*")
    title("Estimated position")
    plot(track[:,1],track[:,2],"b.",track[:,3],track[:,4],"r-",track[:,5],track[:,6],"r-")
    axis("equal")
    grid(1)
    # HERE YOU CAN CHOOSE TO PLOT DIFFERENT DATA:
    # CURRENT HEADING (PLOTTED BY A LINE)
    for i=1:10:size(pos_info.t,1)
        dir = [cos(pos_info.z[i,10]) sin(pos_info.z[i,10])]
        lin = [pos_info.z[i,6:7]; pos_info.z[i,6:7] + 0.1*dir]
        plot(lin[:,1],lin[:,2],"-+")
    end

    # PREDICTED PATH
    # for i=1:4:size(x_est,1)
    #         z_pred = zeros(11,4)
    #         z_pred[1,:] = x_est[i,:]
    #         for j=2:11
    #             z_pred[j,:] = simModel(z_pred[j-1,:],sol_u[j-1,:,i],0.1,0.125,0.125)
    #         end
    #         plot(z_pred[:,1],z_pred[:,2],"-*")
    # end

    # PREDICTED REFERENCE PATH (DEFINED BY POLYNOM)
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

    # figure()
    # plot(oldTraj[:,6,1,2],oldTraj[:,1:5,1,2],"-x")
    # title("Old Trajectory")
    # legend(["v_x","v_y","psiDot","ePsi","eY"])
    # xlabel("s")
    # grid(1)

    figure()
    ax1=subplot(211)
    title("MPC states and cost")
    plot(t-t0,state)
    legend(["v_x","v_y","psiDot","ePsi","eY","s"])
    grid(1)
    subplot(212,sharex = ax1)
    plot(t-t0,cost)
    grid(1)
    legend(["costZ","costZTerm","constZTerm","derivCost","controlCost","laneCost"])

    figure()
    ax1=subplot(211)
    title("States and inputs")
    plot(t-t0,state[:,1:5])
    legend(["v_x","v_y","psiDot","ePsi","eY"])
    grid(1)
    subplot(212,sharex = ax1)
    plot(cmd_log.t-t0,cmd_log.z)
    grid(1)
    legend(["u","d_f"])
end

function eval_predictions_kin(code::AbstractString)
    # This function helps to evaluate predictions of the *kinematic* model
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    d_rec       = load(log_path_record)
    d_lmpc      = load(log_path_LMPC)

    t           = d_lmpc["t"]
    z           = d_lmpc["state"]
    sol_z       = d_lmpc["sol_z"]
    sol_u       = d_lmpc["sol_u"]
    cmd_lmpc    = d_lmpc["cmd"]
    cmd_log     = d_rec["cmd_log"]              # this is the command how it was received by the simulator
    pos_info    = d_rec["pos_info"]
    step_diff   = d_lmpc["step_diff"]

    t0 = t[1]

    sz = size(z,1)
    N = size(sol_z,1)-1     # number of steps (prediction horizon)
    figure(1)
    plot(pos_info.t-t0,pos_info.z[:,2],"--g")
    plot(t-t0,z[:,5],"-b")
    for i=1:sz
        plot((t[i]-t0):.1:(t[i]-t0+0.1*N),sol_z[:,2,i],"--xr")
    end
    grid("on")
    xlabel("t [s]")
    legend(["Estimate","LMPC estimate","Prediction"])
    title("e_y")

    figure(2)
    plot(pos_info.t-t0,pos_info.z[:,3],"--g")
    plot(t-t0,z[:,4],"-b")
    for i=1:sz
        plot((t[i]-t0):.1:(t[i]-t0+0.1*N),sol_z[:,3,i],"--xr")
    end
    grid("on")
    xlabel("t [s]")
    legend(["Estimate","LMPC estimate","Prediction"])
    title("e_psi")

    figure(3)
    plot(pos_info.t-t0,(pos_info.z[:,8].^2+pos_info.z[:,9].^2).^0.5,"--g")
    plot(t-t0,(z[:,1].^2+z[:,2].^2).^0.5,"-b")
    for i=1:sz
        plot(linspace(t[i]-t0,t[i]-t0+0.1*N,N+1),sol_z[:,4,i],"--xr")
    end
    grid("on")
    xlabel("t [s]")
    legend(["Estimate","LMPC estimate","Prediction"])
    title("v")

    figure(4)
    plot(cmd_log.t-t0,cmd_log.z,"--g")
    plot(t-t0,cmd_lmpc,"-b")
    for i=1:sz
        plot(linspace(t[i]-t0,t[i]-t0+0.1*(N-2),N-1),sol_u[1:N-1,1,i],"--xr")
        plot(linspace(t[i]-t0,t[i]-t0+0.1*(N-3),N-2),sol_u[1:N-2,2,i],"--xr")
    end
    grid("on")

    # Calculate one-step-errors:
    one_step_err = zeros(sz,4)
    for i=1:sz-1
        one_step_err[i,:] = sol_z[2,1:4,i] - [z[i+1,6] z[i+1,5] z[i+1,4] norm(z[i+1,1:2])]
    end

    # Calculate 'real' d_f
    L_b = 0.125
    v_x = real(sqrt(complex((pos_info.z[:,8].^2+pos_info.z[:,9].^2)-pos_info.z[:,11].^2*L_b^2)))
    delta = atan2(pos_info.z[:,11]*0.25,v_x)

    figure(5)
    ax1=subplot(211)
    plot(t-t0,one_step_err)
    grid("on")
    legend(["s","ey","epsi","v"])
    subplot(212,sharex=ax1)
    plot(t-t0,cmd_lmpc)
    plot(cmd_log.t_msg-t0,cmd_log.z)
    plot(cmd_log.t-t0,cmd_log.z,"--")
    plot(pos_info.t-t0,delta)
    grid("on")

end
function eval_predictions(code::AbstractString)
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    d_rec       = load(log_path_record)
    d_lmpc      = load(log_path_LMPC)

    t           = d_lmpc["t"]
    z           = d_lmpc["state"]
    sol_z       = d_lmpc["sol_z"]
    sol_u       = d_lmpc["sol_u"]
    cmd_log     = d_rec["cmd_log"]              # this is the command how it was received by the simulator
    pos_info    = d_rec["pos_info"]
    step_diff   = d_lmpc["step_diff"]

    t0 = t[1]

    N = size(sol_z,1)-1     # number of steps (prediction horizon)

    #N = 7
    sz = size(t,1)
    # Calculate one-step-errors:
    ###########
    # s
    # eY
    # ePsi
    # v
    ###########
    # v_x
    # v_y
    # psiDot
    # ePsi
    # eY
    # s
    ###########
    one_step_err = NaN*ones(sz,6)
    for i=1:sz-1
        if isnan(sol_z[1,5,i])          # if we are in path following mode
            one_step_err[i,1:4] = sol_z[2,1:4,i] - [z[i+1,[6,5,4]] norm(z[i+1,1:2])]
        else
            one_step_err[i,:] = sol_z[2,[6,5,4,1,2,3],i] - z[i+1,[6,5,4,1,2,3]]
        end
    end
    i=1
    while isnan(sol_z[1,5,i])
        i = i+1
    end
    figure(1)
    plot(t-t0,step_diff)
    grid("on")
    title("One-step-errors")
    legend(["xDot","yDot","psiDot","ePsi","eY"])
    

    figure(3)
    plot(pos_info.t-t0,pos_info.z[:,11],"-o")
    title("Open loop predictions psidot")
    for i=1:1:size(t,1)-N
        if sol_z[1,5,i]==NaN
            #plot(t[i:i+N]-t0,sol_z[:,2,i],"+")
        else
            plot(t[i:i+N]-t0,sol_z[:,3,i],"-x")
        end
    end
    grid("on")

    figure(4)
    plot(pos_info.t-t0,pos_info.z[:,9],"-o")
    title("Open loop predictions v_y")
    for i=1:1:size(t,1)-N
        if sol_z[1,5,i]==NaN
            #plot(t[i:i+N]-t0,sol_z[:,2,i],"+")
        else
            plot(t[i:i+N]-t0,sol_z[:,2,i],"-x")
        end
    end
    grid("on")


    figure(2)
    ax1=subplot(411)
    title("Open loop predictions e_y")
    plot(pos_info.t-t0,pos_info.z[:,2],"-o")
    for i=1:2:size(t,1)-N
        if sol_z[1,5,i]==NaN
            plot(t[i:i+N]-t0,sol_z[:,2,i],"-+")
        else
            plot(t[i:i+N]-t0,sol_z[:,5,i])
        end
    end
    grid("on")

    subplot(412,sharex=ax1)
    title("Open loop predictions e_psi")
    plot(pos_info.t-t0,pos_info.z[:,3],"-o")
    for i=1:2:size(t,1)-N
        if sol_z[1,5,i]==NaN
            plot(t[i:i+N]-t0,sol_z[:,3,i],"-+")
        else
            plot(t[i:i+N]-t0,sol_z[:,4,i])
        end
    end
    grid("on")

    subplot(413,sharex=ax1)
    title("Open loop predictions v")
    plot(pos_info.t-t0,pos_info.z[:,8],"-o")
    for i=1:2:size(t,1)-N
        if sol_z[1,5,i]==NaN
            plot(t[i:i+N]-t0,sol_z[:,4,i],"-+")
        else
            plot(t[i:i+N]-t0,sol_z[:,1,i])
        end
    end
    grid("on")

    N = 7
    subplot(414,sharex=ax1)
    title("Open loop inputs")
    for i=1:2:size(t,1)-N
        plot(t[i:i+N-3]-t0,sol_u[1:N-2,2,i],t[i:i+N-1]-t0,sol_u[1:N,1,i])
    end
    grid("on")
end


function eval_sysID(code::AbstractString)
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    d_rec       = load(log_path_record)
    d_lmpc      = load(log_path_LMPC)

    oldTraj     = d_lmpc["oldTraj"]
    t           = d_lmpc["t"]
    state       = d_lmpc["state"]
    sol_z       = d_lmpc["sol_z"]
    sol_u       = d_lmpc["sol_u"]
    cost        = d_lmpc["cost"]
    curv        = d_lmpc["curv"]
    c_Vx        = d_lmpc["c_Vx"]
    c_Vy        = d_lmpc["c_Vy"]
    c_Psi       = d_lmpc["c_Psi"]
    cmd         = d_lmpc["cmd"]                 # this is the command how it was sent by the MPC
    step_diff   = d_lmpc["step_diff"]           # this is the command how it was sent by the MPC

    x_est       = d_lmpc["x_est"]
    coeffX      = d_lmpc["coeffX"]
    coeffY      = d_lmpc["coeffY"]
    imu_meas    = d_rec["imu_meas"]
    gps_meas    = d_rec["gps_meas"]
    cmd_log     = d_rec["cmd_log"]              # this is the command how it was received by the simulator
    pos_info    = d_rec["pos_info"]

    t0 = imu_meas.t[1]

    figure(1)       # longitudinal (xDot)
    ax1=subplot(211)
    title("Vx")
    plot(t-t0,c_Vx)
    legend(["c1","c2","c3"])
    grid("on")
    subplot(212,sharex=ax1)
    plot(cmd_log.t-t0,cmd_log.z[:,1])
    grid("on")

    figure(2)       # longitudinal (xDot)
    ax2=subplot(211)
    title("Psi")
    plot(t-t0,c_Psi)
    legend(["c1","c2","c3"])
    grid("on")
    subplot(212,sharex=ax2)
    plot(cmd_log.t-t0,cmd_log.z[:,2])
    grid("on")

    figure(3)       # longitudinal (xDot)
    ax3=subplot(211)
    title("Vy")
    plot(t-t0,c_Vy)
    legend(["c1","c2","c3","c4"])
    grid("on")
    subplot(212,sharex=ax3)
    plot(cmd_log.t-t0,cmd_log.z[:,2])
    grid("on")
end


function eval_oldTraj(code::AbstractString,i::Int64)
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    d = load(log_path_LMPC)
    oldTraj = d["oldTraj"]
    #t       = d["t"]
    plot(oldTraj.oldTimes[:,i],oldTraj.oldTraj[:,:,i],"-x")
    grid("on")
    legend(["v_x","v_x","psiDot","ePsi","eY","s"])
    figure()
    plot(oldTraj.oldTimes[:,i],oldTraj.oldInput[:,:,i],"-x")
    grid("on")
    legend(["a","d_f"])
    figure()
    plot(oldTraj.oldTraj[:,6,i],oldTraj.oldTraj[:,1:5,i],"-x")
    grid("on")
    legend(["v_x","v_x","psiDot","ePsi","eY"])
end

function eval_LMPC_coeff(code::AbstractString,k::Int64)
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    d           = load(log_path_LMPC)
    oldTraj     = d["oldTraj"]
    sol_z       = d["sol_z"]
    sol_u       = d["sol_u"]
    coeffCost   = d["coeffCost"]
    coeffConst  = d["coeffConst"]
    cost        = d["cost"]

    s   = sol_z[:,6,k]
    ss  = [s.^5 s.^4 s.^3 s.^2 s.^1 s.^0]
    subplot(311)
    plot(s,sol_z[:,5,k],"-o",s,ss*coeffConst[:,1,5,k],s,ss*coeffConst[:,2,5,k])
    grid()
    title("Position = $(s[1]), k = $k")
    xlabel("s")
    ylabel("e_Y")
    subplot(312)
    plot(s,sol_z[:,4,k],"-o",s,ss*coeffConst[:,1,4,k],s,ss*coeffConst[:,2,4,k])
    grid()
    xlabel("s")
    ylabel("e_Psi")
    subplot(313)
    plot(s,sol_z[:,1,k],"-o",s,ss*coeffConst[:,1,1,k],s,ss*coeffConst[:,2,1,k])
    grid()
    xlabel("s")
    ylabel("v_x")
    println("Cost = $(cost[k,3])")
end

function anim_LMPC_coeff(code::AbstractString)
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    d           = load(log_path_LMPC)
    oldTraj     = d["oldTraj"]
    sol_z       = d["sol_z"]
    sol_u       = d["sol_u"]
    coeffCost   = d["coeffCost"]
    coeffConst  = d["coeffConst"]
    cost        = d["cost"]
    sol_status  = d["sol_status"]

    i=1
    while isnan(sol_z[1,6,i])
        i = i+1
    end
    for k=i:size(cost,1)
        clf()
        s   = sol_z[:,6,k]
        ss  = [s.^5 s.^4 s.^3 s.^2 s.^1 s.^0]
        subplot(511)
        plot(s,sol_z[:,5,k],"-o",s,ss*coeffConst[:,1,5,k],s,ss*coeffConst[:,2,5,k])
        ylim([-0.4,0.4])
        grid()
        title("Position = $(s[1]), k = $k, status = $(sol_status[k])")
        xlabel("s")
        ylabel("e_Y")
        subplot(512)
        plot(s,sol_z[:,4,k],"-o",s,ss*coeffConst[:,1,4,k],s,ss*coeffConst[:,2,4,k])
        ylim([-0.5,0.5])
        grid()
        xlabel("s")
        ylabel("e_Psi")
        subplot(513)
        plot(s,sol_z[:,3,k],"-o",s,ss*coeffConst[:,1,3,k],s,ss*coeffConst[:,2,3,k])
        ylim([-0.5,0.5])
        grid()
        xlabel("s")
        ylabel("psiDot")
        subplot(514)
        plot(s,sol_z[:,1,k],"-o",s,ss*coeffConst[:,1,1,k],s,ss*coeffConst[:,2,1,k])
        ylim([0.6,1.5])
        grid()
        xlabel("s")
        ylabel("v_x")
        subplot(515)
        plot(s,sol_z[:,2,k],"-o",s,ss*coeffConst[:,1,2,k],s,ss*coeffConst[:,2,2,k])
        ylim([-0.5,0.5])
        grid()
        xlabel("s")
        ylabel("v_y")
        println("Cost = $(cost[k,3])")
    end
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

function visualize_tdiff(code::AbstractString)
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    d_rec       = load(log_path_record)
    pos_info    = d_rec["pos_info"]
    t_diff = diff(pos_info.t_msg)
    t_diff = t_diff[t_diff .< 0.08]
    plt[:hist](t_diff,100)
    grid("on")
    xlabel("t_diff")
    nothing
end

function anim_run(code::AbstractString)
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    d_rec       = load(log_path_record)
    pos_info    = d_rec["pos_info"]
    L_a = 0.125
    w = 0.15
    alpha = atan(w/2/L_a)
    l = sqrt(L_a^2+(w/2)^2)
    t0 = pos_info.t[1]
    #Construct Figure and Plot Data
    fig = figure(figsize=(10,10))
    ax = axes(xlim = (-3,3),ylim=(-5,1))

    track = create_track(0.4)
    plot(track[:,1],track[:,2],"b.",track[:,3],track[:,4],"r-",track[:,5],track[:,6],"r-")
    grid("on")
    car = ax[:plot]([],[],"r-+")[1]
    gps = ax[:plot]([],[],"g*")[1]
    h_ti = ax[:title]("abc")
    function init()
        car[:set_data]([],[])
        return (car,None)
    end
    function animate(k)
        i = k + 2000
        car_c = [pos_info.z[i,6]+cos(pos_info.z[i,10]-alpha)*l pos_info.z[i,7]+sin(pos_info.z[i,10]-alpha)*l;
                pos_info.z[i,6]+cos(pos_info.z[i,10]+alpha)*l pos_info.z[i,7]+sin(pos_info.z[i,10]+alpha)*l;
                pos_info.z[i,6]+cos(pos_info.z[i,10]+pi-alpha)*l pos_info.z[i,7]+sin(pos_info.z[i,10]+pi-alpha)*l;
                pos_info.z[i,6]+cos(pos_info.z[i,10]+pi+alpha)*l pos_info.z[i,7]+sin(pos_info.z[i,10]+pi+alpha)*l;
                pos_info.z[i,6]+cos(pos_info.z[i,10]-alpha)*l pos_info.z[i,7]+sin(pos_info.z[i,10]-alpha)*l]
        car[:set_data]([car_c[:,1]],[car_c[:,2]])
        gps[:set_data]([pos_info.z[max(1,i-100):i,12]],[pos_info.z[max(1,i-100):i,13]])
        ax[:set_title]("t = $(pos_info.t[i]-t0)")
        #title(pos_info.t[i]-t0)
        return (car,gps,None)
    end
    t=0:30
    anim = animation.FuncAnimation(fig, animate, frames=1000, interval=50)
    anim[:save]("test2.mp4", bitrate=-1, extra_args=["-vcodec", "libx264", "-pix_fmt", "yuv420p"]);
end

function anim_constraints(code::AbstractString)
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    d_rec       = load(log_path_record)
    d_lmpc      = load(log_path_LMPC)

    t           = d_lmpc["t"]
    sol_z       = d_lmpc["sol_z"]
    sol_u       = d_lmpc["sol_u"]
    coeffCost   = d_lmpc["coeffCost"]
    coeffConst  = d_lmpc["coeffConst"]

    #Construct Figure and Plot Data
    fig = figure(figsize=(10,10))
    ax = axes()

    pred_vx = ax[:plot]([],[],"r-+")[1]
    poly_vx1 = ax[:plot]([],[],"--")[1]
    poly_vx2 = ax[:plot]([],[],"--")[1]

    function init()
        #car[:set_data]([],[])
        return (None)
    end
    function animate(k)
        i = k + 1000
        s = sol_z[:,6,i]
        ss  = [s.^5 s.^4 s.^3 s.^2 s.^1 s.^0]
        pred_vx[:set_data]([s],[sol_z[:,1,i]])
        poly_vx1[:set_data]([s],[ss*coeffConst[:,1,1,i]])
        poly_vx2[:set_data]([s],[ss*coeffConst[:,2,1,i]])

        return (pred_vx,poly_vx1,poly_vx2,None)
    end
    t=0:30
    anim = animation.FuncAnimation(fig, animate, frames=100, interval=50)
    anim[:save]("test2.mp4", bitrate=-1, extra_args=["-vcodec", "libx264", "-pix_fmt", "yuv420p"]);
end
# *****************************************************************
# ****** HELPER FUNCTIONS *****************************************
# *****************************************************************

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

# function eval_prof()
#     Profile.clear()
#     @load "$(homedir())/simulations/profile.jlprof"
#     ProfileView.view(li, lidict=lidict)
# end

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

    # # SIMPLE track
    # add_curve(theta,50,0)
    # add_curve(theta,100,-pi)
    # add_curve(theta,100,0)
    # add_curve(theta,100,-pi)
    # add_curve(theta,49,0)

    # GOGGLE TRACK
    # add_curve(theta,30,0)
    # add_curve(theta,40,-pi/2)
    # add_curve(theta,40,-pi/2)
    # add_curve(theta,20,-pi/6)
    # add_curve(theta,30,pi/3)
    # add_curve(theta,20,-pi/6)
    # add_curve(theta,40,-pi/2)
    # add_curve(theta,40,-pi/2)
    # add_curve(theta,35,0)

    # SIMPLE GOGGLE TRACK
    add_curve(theta,30,0)
    add_curve(theta,40,-pi/2)
    add_curve(theta,40,-pi/2)
    add_curve(theta,20,-pi/10)
    add_curve(theta,30,pi/5)
    add_curve(theta,20,-pi/10)
    add_curve(theta,40,-pi/2)
    add_curve(theta,40,-pi/2)
    add_curve(theta,35,0)

    #  # SHORT SIMPLE track
    # add_curve(theta,10,0)
    # add_curve(theta,80,-pi)
    # add_curve(theta,20,0)
    # add_curve(theta,80,-pi)
    # add_curve(theta,9,0)

    for i=1:length(theta)
            push!(x, x[end] + cos(theta[i])*ds)
            push!(y, y[end] + sin(theta[i])*ds)
            push!(x_l, x[end-1] + cos(theta[i]+pi/2)*w)
            push!(y_l, y[end-1] + sin(theta[i]+pi/2)*w)
            push!(x_r, x[end-1] + cos(theta[i]-pi/2)*w)
            push!(y_r, y[end-1] + sin(theta[i]-pi/2)*w)
    end
    track = cat(2, x, y, x_l, y_l, x_r, y_r)
    #plot(track[:,1],track[:,2])
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

function initPlot()
    linewidth = 0.4
    rc("axes", linewidth=linewidth)
    rc("lines", linewidth=linewidth, markersize=2)
    #rc("font", family="")
    rc("axes", titlesize="small", labelsize="small")        # can be set in Latex
    rc("xtick", labelsize="x-small")
    rc("xtick.major", width=linewidth/2)
    rc("ytick", labelsize="x-small")
    rc("ytick.major", width=linewidth/2)
    rc("legend", fontsize="small")
    rc("font",family="serif")
    rc("font",size=8)
    rc("figure",figsize=[4.5,3])
    #rc("pgf", texsystem="pdflatex",preamble=L"""\usepackage[utf8x]{inputenc}\usepackage[T1]{fontenc}\usepackage{lmodern}""")
end

function simModel(z,u,dt,l_A,l_B)

   # kinematic bicycle model
   # u[1] = acceleration
   # u[2] = steering angle

    bta = atan(l_A/(l_A+l_B)*tan(u[2]))

    zNext = copy(z)
    zNext[1] = z[1] + dt*(z[4]*cos(z[3] + bta))                 # x
    zNext[2] = z[2] + dt*(z[4]*sin(z[3] + bta))                 # y
    zNext[3] = z[3] + dt*(z[4]/l_B*sin(bta))                    # psi
    zNext[4] = z[4] + dt*(u[1] - 0.5 * z[4])                    # v

    return zNext
end

function checkTimes(code::AbstractString)
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    d_lmpc      = load(log_path_LMPC)

    t_solv     = d_lmpc["t_solv"]
    sol_status = d_lmpc["sol_status"]
    cmd        = d_lmpc["cmd"]                 # this is the command how it was sent by the MPC
    sol_status_int = zeros(size(t_solv,1))
    t = d_lmpc["t"]

    for i=1:size(sol_status,1)
        if sol_status[i]==:Optimal
            sol_status_int[i] = 1
        elseif sol_status[i]==:Infeasible
            sol_status_int[i] = 2
        elseif sol_status[i]==:UserLimit
            sol_status_int[i] = 3
        end
    end
    ax1=subplot(211)
    plot(t,t_solv)
    plot(t,sol_status_int,"*")
    grid("on")
    subplot(212,sharex=ax1)
    plot(t,cmd,"-x")
    grid("on")
end

function checkConnectivity(code::AbstractString)
    log_path_record = "$(homedir())/open_loop/output-record-$code.jld"
    d_rec = load(log_path_record)

    cmd_pwm_log = d_rec["cmd_pwm_log"]
    vel_est     = d_rec["vel_est"]
    pos_info    = d_rec["pos_info"]
    imu_meas    = d_rec["imu_meas"]
    gps_meas    = d_rec["gps_meas"]

    plot(gps_meas.t,gps_meas.z,"-x",gps_meas.t_msg,gps_meas.z,"--x",pos_info.t,pos_info.z[:,12:13],"-*",pos_info.t_msg,pos_info.z[:,12:13],"--*")
    grid("on")
end

function smooth(x,n)
    y = zeros(size(x))
    for i=1:size(x,1)
        start = max(1,i-n)
        fin = min(size(x,1),start + 2*n)
        y[i,:] = mean(x[start:fin,:],1)
    end
    return y
end