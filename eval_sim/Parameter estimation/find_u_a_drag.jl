
using PyPlot
using JLD

function main(code::AbstractString)
    log_path_record = "$(homedir())/open_loop/output-record-$(code).jld"
    d_rec = load(log_path_record)
    L_b = 0.125

    imu_meas    = d_rec["imu_meas"]
    gps_meas    = d_rec["gps_meas"]
    cmd_log     = d_rec["cmd_log"]
    cmd_pwm_log = d_rec["cmd_pwm_log"]
    vel_est     = d_rec["vel_est"]
    pos_info    = d_rec["pos_info"]

    t0 = max(cmd_pwm_log.t[1],vel_est.t[1],imu_meas.t[1])
    t_end = min(cmd_pwm_log.t[end],vel_est.t[end],imu_meas.t[end])

    t = t0+0.1:.02:t_end-0.1
    v = zeros(length(t))
    cmd = zeros(length(t))
    cmd_raw = zeros(length(t))

    for i=1:length(t)
        if cmd_log.z[t[i].>cmd_log.t,1][end] > 0 && vel_est.z[t[i].>vel_est.t,1][end] > 0.2
            v[i] = vel_est.z[t[i].>vel_est.t,1][end]
            cmd[i] = cmd_log.z[t[i].>cmd_log.t,1][end]
            cmd_raw[i] = cmd_pwm_log.z[t[i].>cmd_pwm_log.t,1][end]
        end
    end

    idx = (v.>1) & (cmd_raw .> 95)

    sz = size(v[idx],1)
    A = [cmd_raw[idx] ones(sz,1)]
    b = v[idx]
    coeff = A\b

    x = [90,110]
    y = [90 1;110 1]*coeff

    #plot(cmd,v)
    figure(1)
    ax1=subplot(211)
    plot(vel_est.t,vel_est.z)
    grid("on")
    legend(["v","v_fl","v_fr","v_bl","v_br"])
    subplot(212,sharex=ax1)
    plot(cmd_pwm_log.t,cmd_pwm_log.z[:,1])
    grid("on")

    figure(2)
    plot(cmd[cmd.>0],cmd[cmd.>0]./v[cmd.>0],"*")
    xlabel("u_a")
    ylabel("u_a/v")
    grid("on")

    figure(3)
    plot(cmd_raw[cmd_raw.>80],v[cmd_raw.>80],"*",x,y)
    grid("on")
    xlabel("PWM signal")
    ylabel("v [m/s]")

    println("c1 = $(1/coeff[1])")
    println("c2 = $(coeff[2]/coeff[1])")
end
