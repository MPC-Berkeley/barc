# This script can be used to simulate the onboard-Kalman filter using real recorded sensor data.
# This can be useful to find the optimal Kalman parameters (Q,R) or to just do experiments and find an optimal sensor fusion process.

using PyPlot
using JLD

Q = diagm([0.1,0.1,0.1,0.1,0.1,0.1,0.1])
R = diagm([0.1,0.1,0.1,1.0,10.0,10.0])

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

    t0      = max(imu_meas.t[1],vel_est.t[1],gps_meas.t[1])+0.3
    t_end   = min(imu_meas.t[end],vel_est.t[end],gps_meas.t[end],cmd_pwm_log.t[end])-0.3

    l_A = 0.125
    l_B = 0.125

    dt = 1/50
    t = t0:dt:t_end
    sz = length(t)
    y = zeros(sz,6)
    u = zeros(sz,2)

    y_gps_imu = zeros(sz,5)

    x_est = zeros(length(t),6)
    P_gps_imu = zeros(6,6)
    x_est_gps_imu = zeros(length(t),8)
    #x_est_gps_imu[1,11] = 0.5
    #x_est_gps_imu[1,10] = -0.6

    yaw0 = imu_meas.z[1,6]
    gps_dist = zeros(length(t))

    yaw_prev = yaw0

    Q_gps_imu = diagm([1.0,1.0,1.0,0.1,0.1,0.01])
    R_gps_imu = diagm([1.0,1.0,0.1,0.1,5.0])

    gps_gate = zeros(length(t))

    gps_prev = gps_meas.z[1,:]
    vel_est_prev = vel_est.z[1,1]
    imu_prev = imu_meas.z[1,6]
    # measured: IMU_a
    # calibrated: Z_a
    # need matrix Z_IMU_A
    for i=2:length(t)
        # Collect measurements and inputs for this iteration
        y_gps = gps_meas.z[t[i].>gps_meas.t,:][end,:]
        y_yaw = imu_meas.z[t[i].>imu_meas.t,6][end]-yaw0
        y_yawdot = imu_meas.z[t[i].>imu_meas.t,3][end]
        a_x = imu_meas.z[t[i].>imu_meas.t,7][end]
        a_y = imu_meas.z[t[i].>imu_meas.t,8][end]
        y_vel_est = vel_est.z[t[i].>vel_est.t,1][end]

        y_gps_imu[i,:] = [y_gps y_yaw y_vel_est y_yawdot]
        y_gps_imu[:,3] = unwrap!(y_gps_imu[:,3])

        u[i,1] = cmd_log.z[t[i].>cmd_log.t,1][end]
        u[i,2] = cmd_log.z[t[i].>cmd_log.t-0.2,2][end]

        gps_dist[i] = norm(y[i,1:2]-x_est[i-1,1:2])
        #gps_gate[i] = 
        # if gps_dist[i] > 1.0
        #     R[1,1] = 1+10*gps_dist[i]^2
        #     R[2,2] = 1+10*gps_dist[i]^2
        #     R_gps_imu[1,1] = 1.0+100*gps_dist[i]^2
        #     R_gps_imu[2,2] = 1.0+100*gps_dist[i]^2
        # else
        #     R[1,1] = 1
        #     R[2,2] = 1
        #     R_gps_imu[1,1] = 0.1
        #     R_gps_imu[2,2] = 0.1
        # end
        if gps_prev == y_gps
            R_gps_imu[1:2,1:2] = 100.0*eye(2)
        else
            R_gps_imu[1:2,1:2] = 10.0*eye(2)
        end
        if vel_est_prev == y_vel_est
            R_gps_imu[4,4] = 10.0
        else
            R_gps_imu[4,4] = 0.1
        end
        if imu_prev == y_yaw
            R_gps_imu[3,3] = 100000.0
            R_gps_imu[5,5] = 10.0
        else
            R_gps_imu[3,3] = 0.1
            R_gps_imu[5,5] = 5.0
        end
        gps_prev = y_gps
        vel_est_prev = y_vel_est
        imu_prev = y_yaw
        args = (u[i,:],dt,l_A,l_B)

        # Calculate new estimate
        (x_est_gps_imu[i,1:6], P_gps_imu, gps_gate[i]) = ekf_gate(simModel_gps_imu,x_est_gps_imu[i-1,1:6]',P_gps_imu,h_gps_imu,y_gps_imu[i,:]',Q_gps_imu,R_gps_imu,args)
        beta = atan(l_A*tan(u[i,2])/(l_B+l_A))
        x_est_gps_imu[i,7] = cos(beta)*x_est_gps_imu[i,4]     # v_x
        x_est_gps_imu[i,8] = l_B*x_est_gps_imu[i,5]     # v_y
    end

    figure(1)
    plot(t-t0,x_est_gps_imu[:,1:2],"-*",gps_meas.t-t0,gps_meas.z,"-x",pos_info.t-t0,pos_info.z[:,6:7],"--")
    grid("on")
    title("Comparison x,y estimate and measurement")
    legend(["x_est","y_est","x_meas","y_meas","x_onboard","y_onboard"])

    figure(2)
    plot(t-t0,x_est_gps_imu[:,6])
    grid("on")
    title("Drifts")
    legend(["psi"])

    figure(3)
    v = sqrt(x_est_gps_imu[:,3].^2+x_est_gps_imu[:,4].^2)
    plot(t-t0,v,"-*",t-t0,x_est_gps_imu[:,3:4],"--",vel_est.t-t0,vel_est.z,pos_info.t-t0,pos_info.z[:,8:9],"-x")
    grid("on")
    legend(["v","v_x_est","v_y_est","v_meas","v_x_onboard","v_y_onboard"])
    title("Velocity estimate and measurement")

    figure(4)
    plot(t-t0,x_est_gps_imu[:,3],"-*",t-t0,y_gps_imu[:,3])
    grid("on")
    title("Comparison yaw")
    legend(["psi_est","psi_meas"])

    figure(5)
    plot(imu_meas.t-t0,imu_meas.z[:,3],t-t0,x_est_gps_imu[:,5])
    title("w_z")
    legend(["w_z_meas","w_z_est"])
    grid("on")

    figure(6)
    plot(t-t0,x_est_gps_imu[:,7:8])
    grid("on")
    legend(["v_x_est","v_y_est"])

    figure(7)
    plot(t-t0,gps_gate,t-t0,gps_dist,gps_meas.t-t0,gps_meas.z)
    grid("on")
    legend(["gate","dist"])

    # # CORRELATIONS:
    # figure(8)
    # plot(94.14+2.7678*u[:,1],y[:,4],"*")
    # grid("on")
    # title("Comparison motor input and velocity")
    # xlabel("PWM signal")
    # ylabel("v [m/s]")

    # delta = atan(x_est_gps_imu[:,8]*0.25./x_est_gps_imu[:,3])
    # #delta2 = atan(0.25/sqrt(y_gps_imu[:,3].^2./y_gps_imu[:,5].^2-0.125^2))
    # figure(9)
    # plot(u[:,2],delta,"*")
    # grid("on")

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

function h_gps_imu(x,args)
    y = zeros(5)
    y[1] = x[1]                     # x
    y[2] = x[2]                     # y
    y[3] = x[3]+x[6]                # psi
    y[4] = x[4]                     # v
    y[5] = x[5]                     # psiDot
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

function ekf_gate(f, mx_k, P_k, h, y_kp1, Q, R, args)
    xDim    = size(mx_k,1)
    mx_kp1  = f(mx_k,args)
    A       = numerical_jac(f,mx_k,args)
    P_kp1   = A*P_k*A' + Q
    my_kp1  = h(mx_kp1,args)
    H       = numerical_jac(h,mx_kp1,args)
    P12     = P_kp1*H'
    S       = H*P12
    gps_err = ((y_kp1[1:2]-my_kp1[1:2])'*inv(S[1:2,1:2])*(y_kp1[1:2]-my_kp1[1:2]))[1]
    # if gps_err > 100.0
    #     R[1:2,1:2] = eye(2)*100
    # else
    #     R[1:2,1:2] = eye(2)*0.1
    # end
    K       = P12*inv(H*P12+R)
    mx_kp1  = mx_kp1 + K*(y_kp1-my_kp1)
    P_kp1   = (K*R*K' + (eye(xDim)-K*H)*P_kp1)*(eye(xDim)-K*H)'
    return mx_kp1, P_kp1, gps_err
end

function simModel_gps_imu(z,args)
    (u,dt,l_A,l_B) = args
    #bta = atan(l_A/(l_A+l_B)*tan(u[2]))
    bta = atan2(l_A*tan(u[2]),l_A+l_B)
    zNext = copy(z)
    zNext[1] = z[1] + dt*(z[4]*cos(z[3] + bta))                     # x
    zNext[2] = z[2] + dt*(z[4]*sin(z[3] + bta))                     # y
    zNext[3] = z[3] + dt*(z[4]/l_B*sin(bta))                        # psi
    zNext[4] = z[4] + dt*(u[1]-1.0*z[4])                            # v
    zNext[5] = z[5]                                                 # psiDot
    zNext[6] = z[6]                                                 # drift_psi
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

function rotMatrix(s::Char,deg::Float64)
    A = zeros(3,3)
    if s=='x'
        A = [1 0 0;
             0 cos(deg) sin(deg);
             0 -sin(deg) cos(deg)]
    elseif s=='y'
        A = [cos(deg) 0 -sin(deg);
             0 1 0;
             sin(deg) 0 cos(deg)]
    elseif s=='z'
        A = [cos(deg) sin(deg) 0
             -sin(deg) cos(deg) 0
             0 0 1]
    else
        warn("Wrong angle for rotation matrix")
    end
    return A
end

function angRates2EulerRates(w::Array{Float64},deg::Array{Float64})
    (p,q,r) = w
    (phi,theta,psi) = deg
    phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r
    thetadot = cos(phi)*q - sin(phi)*r
    psidot = sin(phi)*sec(theta)*q + cos(phi)*sec(theta)*r
    return [phidot,thetadot,psidot]
end

function eulerTrans(deg::Array{Float64})    # Transformation matrix inertial -> body
    return rotMatrix('x',deg[1])*rotMatrix('y',deg[2])*rotMatrix('z',deg[3])
end

function getIndex(t_raw::Array{Float64},t::Float64)
    i = 1
    while i<=length(t_raw)
        if t > t_raw[i]
            i += 1
        else
            break
        end
    end
    return i
end