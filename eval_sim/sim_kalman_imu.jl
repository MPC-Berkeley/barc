# This script can be used to simulate the onboard-Kalman filter using real recorded sensor data.
# This can be useful to find the optimal Kalman parameters (Q,R) or to just do experiments and find an optimal sensor fusion process.

using PyPlot
using JLD

Q = diagm([0.1,0.1,0.1,0.1,0.1,0.1,0.1])
R = diagm([0.1,0.1,0.1,1.0,10.0,10.0])

R_gps_imu = diagm([0.1,0.1,1.0,1.0,0.1,0.1,0.01])

function main(code::AbstractString)
    global Q, R, R_gps_imu
    log_path_record = "$(homedir())/open_loop/output-record-$(code).jld"
    d_rec = load(log_path_record)

    imu_meas    = d_rec["imu_meas"]
    gps_meas    = d_rec["gps_meas"]
    cmd_log     = d_rec["cmd_log"]
    cmd_pwm_log = d_rec["cmd_pwm_log"]
    vel_est     = d_rec["vel_est"]
    pos_info    = d_rec["pos_info"]

    t0      = max(imu_meas.t[1],vel_est.t[1],gps_meas.t[1])
    t_end   = min(imu_meas.t[end],vel_est.t[end],gps_meas.t[end],cmd_pwm_log.t[end])

    l_A = 0.125
    l_B = 0.125

    dt = 1/25
    t = t0:dt:t_end
    sz = length(t)
    y = zeros(sz,6)
    u = zeros(sz,2)

    y_gps_imu = zeros(sz,7)

    P = zeros(7,7)
    x_est = zeros(length(t),7)
    P_gps_imu = zeros(11,11)
    x_est_gps_imu = zeros(length(t),11)

    yaw0 = imu_meas.z[1,6]
    gps_dist = zeros(length(t))

    yaw_prev = yaw0

    Q_gps_imu = diagm([1/6*dt^3,1/6*dt^3,1/2*dt^2,1/2*dt^2,dt,dt,dt,dt,0.001,0.001,0.001])

    for i=2:length(t)
        # Collect measurements and inputs for this iteration
        y_gps = gps_meas.z[gps_meas.t.>t[i],:][1,:]
        y_yaw = imu_meas.z[imu_meas.t.>t[i],6][1]-yaw0
        y_yawdot = imu_meas.z[imu_meas.t.>t[i],3][1]
        a_x   = imu_meas.z[imu_meas.t.>t[i],7][1]
        a_y   = imu_meas.z[imu_meas.t.>t[i],8][1]
        y_vel_est = vel_est.z[vel_est.t.>t[i]][1]

        y[i,:] = [y_gps y_yaw y_vel_est a_x a_y]
        y_gps_imu[i,:] = [y_gps a_x a_y y_yaw y_yawdot y_vel_est]
        y[:,3] = unwrap!(y[:,3])
        y_gps_imu[:,5] = unwrap!(y_gps_imu[:,5])

        u[i,:] = cmd_log.z[cmd_log.t.>t[i],:][1,:]

        # Adapt R-value of GPS according to distance to last point:
        gps_dist[i] = norm(y[i,1:2]-x_est[i-1,1:2])
        if gps_dist[i] > 0.3
            R[1,1] = 1+100*gps_dist[i]^2
            R[2,2] = 1+100*gps_dist[i]^2
            R_gps_imu[1,1] = 1+100*gps_dist[i]^2
            R_gps_imu[2,2] = 1+100*gps_dist[i]^2
        else
            R[1,1] = 1
            R[2,2] = 1
            R_gps_imu[1,1] = 1
            R_gps_imu[2,2] = 1
        end

        
        args = (u[i,:],dt,l_A,l_B)

        # Calculate new estimate
        (x_est[i,:], P) = ekf(simModel,x_est[i-1,:]',P,h,y[i,:]',Q,R,args)
        (x_est_gps_imu[i,:], P_gps_imu) = ekf(simModel_gps_imu,x_est_gps_imu[i-1,:]',P_gps_imu,h_gps_imu,y_gps_imu[i,:]',Q_gps_imu,R_gps_imu,args)
    end

    figure(1)
    plot(t-t0,x_est_gps_imu[:,1:2],"-*",gps_meas.t-t0,gps_meas.z)
    grid("on")
    title("Comparison x,y estimate and measurement")

    figure(2)
    plot(t-t0,x_est_gps_imu[:,9:11])
    grid("on")
    title("Drifts")
    legend(["a_x","a_y","psi"])

    figure(3)
    v = sqrt(x_est_gps_imu[:,3].^2+x_est_gps_imu[:,4].^2)
    plot(t-t0,v,"-*",t-t0,x_est_gps_imu[:,3:4],"--",vel_est.t-t0,vel_est.z)
    grid("on")
    title("Velocity estimate and measurement")

    # ax1=subplot(211)
    # plot(t,y,"-x",t,x_est,"-*")
    # grid("on")
    # legend(["x","y","psi","v"])
    # subplot(212,sharex=ax1)
    # plot(t,u)
    # grid("on")
    # legend(["a_x","d_f"])
    # figure(1)
    # ax=subplot(5,1,1)
    # for i=1:4
    #     subplot(5,1,i,sharex=ax)
    #     plot(t,y[:,i],t,x_est[:,i],"-*")
    #     grid("on")
    # end
    # subplot(5,1,5,sharex=ax)
    # plot(cmd_pwm_log.t,cmd_pwm_log.z)
    # grid("on")


    # figure(2)
    # subplot(2,1,1)
    # title("Comparison raw GPS data and estimate")
    # plot(t-t0,y[:,1],t-t0,x_est[:,1],"-*")
    # xlabel("t [s]")
    # ylabel("Position [m]")
    # legend(["x_m","x_est"])
    # grid("on")
    # subplot(2,1,2)
    # plot(t-t0,y[:,2],t-t0,x_est[:,2],"-*")
    # xlabel("t [s]")
    # ylabel("Position [m]")
    # legend(["y_m","y_est"])
    # grid("on")

    # figure(3)
    # plot(t,gps_dist)
    # title("GPS distances")
    # grid("on")
    # # figure()
    # # plot(gps_meas.z[:,1],gps_meas.z[:,2],"-x",x_est[:,1],x_est[:,2],"-*")
    # # grid("on")
    # # title("x-y-view")
    # figure(4)
    # plot(t-t0,x_est,"-x",pos_info.t-t0,pos_info.z[:,[6,7,4,10,16]],"-*")
    # title("Comparison simulation (-x) onboard-estimate (-*)")
    # grid("on")
    # legend(["x","y","psi","v","psi_drift"])

    # unwrapped_yaw_meas = unwrap!(imu_meas.z[:,6])
    # figure(5)
    # plot(t-t0,x_est[:,3],"-*",imu_meas.t-t0,unwrapped_yaw_meas-unwrapped_yaw_meas[1])
    # title("Comparison yaw")
    # grid("on")

    # figure(6)
    # plot(t-t0,x_est[:,4],"-*",vel_est.t-t0,vel_est.z)
    # grid("on")
    # title("Comparison of velocity measurement and estimate")
    # legend(["estimate","measurement"])

    #figure(7)
    #plt[:hist](gps_dist,300)
    #grid("on")
    nothing
end

function h(x,args)
    C = [eye(6) zeros(6,1)]
    C[4,4] = 1
    #C[3,3] = 0
    C[3,5] = 1
    C[5,5] = 0
    C[5,6] = 1
    C[6,6] = 0
    C[6,7] = 1
    return C*x
end
function h_gps_imu(x,args)
    C = [1 0 0 0 0 0 0 0 0 0 0;     # x
         0 1 0 0 0 0 0 0 0 0 0;     # y
         0 0 0 0 1 0 0 0 1 0 0;     # a_x
         0 0 0 0 0 1 0 0 0 1 0;     # a_y
         0 0 0 0 0 0 1 0 0 0 1;     # psi
         0 0 0 0 0 0 0 1 0 0 0;     # psi_dot
         0 0 0 0 0 0 0 0 0 0 0]     # v
    y = C*x
    y[7] = sqrt(x[3]^2+x[4]^2)      # Idea: v_x_meas = cos(beta)*v, v_y_meas = sin(beta)*v
    #return C*x
    return y
end

function ekf(f, mx_k, P_k, h, y_kp1, Q, R, args)
    xDim    = size(mx_k,1)
    mx_kp1  = f(mx_k,args)
    A       = numerical_jac(f,mx_k,args)
    P_kp1   = A*P_k*A' + Q
    my_kp1  = h(mx_kp1,args)
    H       = numerical_jac(h,mx_kp1,args)
    P12     = P_kp1*H'
    K       = P12*inv(H*P12+R)
    mx_kp1  = mx_kp1 + K*(y_kp1-my_kp1)
    P_kp1   = (K*R*K' + (eye(xDim)-K*H)*P_kp1)*(eye(xDim)-K*H)'
    return mx_kp1, P_kp1
end

function simModel(z,args)
   # kinematic bicycle model
   # u[1] = acceleration
   # u[2] = steering angle
    (u,dt,l_A,l_B) = args
    bta = atan(l_A/(l_A+l_B)*tan(u[2]))

    zNext = copy(z)
    zNext[1] = z[1] + dt*(z[4]*cos(z[3] + bta))                     # x
    zNext[2] = z[2] + dt*(z[4]*sin(z[3] + bta))                     # y
    zNext[3] = z[3] + dt*(z[4]/l_B*sin(bta))                        # psi
    zNext[4] = z[4] + dt*(sqrt(z[6]^2+z[7]^2))                      # v
    #zNext[4] = z[4] + dt*(z[6])                                    # v
    #zNext[4] = z[4] + dt*(sqrt(z[6]^2+z[7]^2))                     # v
    zNext[5] = z[5]                                                 # psi_drift
    zNext[6] = z[6]                                                 # a_x
    zNext[7] = z[7]                                                 # a_y
    return zNext
end

function simModel_gps_imu(z,args)
    zNext = copy(z)
    (u,dt,l_A,l_B) = args
    bta = atan(l_A/(l_A+l_B)*tan(u[2]))
    zNext[1] = z[1] + dt*(cos(z[7])*z[3] - sin(z[7])*z[4]) + 1/2*dt^2*(cos(z[7])*z[5]-sin(z[7])*z[6])       # x
    zNext[2] = z[2] + dt*(sin(z[7])*z[3] + cos(z[7])*z[4]) + 1/2*dt^2*(sin(z[7])*z[5]+cos(z[7])*z[6])       # y
    #zNext[1] = z[1] + dt*(cos(z[7])*z[3] - sin(z[7])*z[4]) + 1/2*dt^2*(cos(z[7])*(z[5]+z[8]*z[4])-sin(z[7])*(z[6]-z[8]*z[3]))       # x
    #zNext[2] = z[2] + dt*(sin(z[7])*z[3] + cos(z[7])*z[4]) + 1/2*dt^2*(sin(z[7])*(z[5]+z[8]*z[4])+cos(z[7])*(z[6]-z[8]*z[3]))       # y
    zNext[3] = z[3] + dt*(z[5]+z[8]*z[4])   # v_x
    zNext[4] = z[4] + dt*(z[6]-z[8]*z[3])   # v_y
    zNext[5] = z[5]                         # a_x
    zNext[6] = z[6]                         # a_y
    zNext[7] = z[7] + dt*(z[3]/l_B*sin(bta))  # psi
    #zNext[7] = z[7] + dt*z[8]               # psi
    zNext[8] = z[8]                         # psidot
    zNext[9] = z[9]                         # drift_a_x
    zNext[10] = z[10]                       # drift_a_y
    zNext[11] = z[11]                       # drift_psi
    return zNext
end

function numerical_jac(f,x,args)
    y = f(x,args)
    jac = zeros(size(y,1),size(x,1))
    eps = 1e-5
    xp = copy(x)
    for i = 1:size(x,1)
        xp[i] = x[i]+eps/2.0
        yhi = f(xp,args)
        xp[i] = x[i]-eps/2.0
        ylo = f(xp,args)
        xp[i] = x[i]
        jac[:,i] = (yhi-ylo)/eps
    end
    return jac
end

function initPlot()             # Initialize Plots for export
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

function unwrap!(p)
    length(p) < 2 && return p
    for i = 2:length(p)
        d = p[i] - p[i-1]
        if abs(d) > pi
            p[i] -= floor((d+pi) / (2*pi)) * 2pi
        end
    end
    return p
end