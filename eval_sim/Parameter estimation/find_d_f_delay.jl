
using PyPlot
using JLD

function main(code::AbstractString)
    #log_path_record = "$(homedir())/open_loop/output-record-$(code).jld"
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
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
    psiDot = zeros(length(t))

    for i=1:length(t)
        v[i] = vel_est.z[t[i].>vel_est.t,1][end]
        psiDot[i] = imu_meas.z[t[i].>imu_meas.t,3][end]

    end
    v_x = real(sqrt(complex(v.^2-psiDot.^2*L_b^2)))
    v_y = L_b*psiDot
    delta = atan2(psiDot*0.25,v_x)
    figure(1)
    plot(t-t0,delta,cmd_log.t-t0,cmd_log.z[:,2])
    grid("on")
    xlabel("t [s]")
    legend(["delta_true","delta_input"])

    figure(2)
    ax1=subplot(211)
    plot(cmd_pwm_log.t-t0,floor(cmd_pwm_log.z[:,2]))
    grid("on")
    subplot(212,sharex=ax1)
    plot(vel_est.t-t0,vel_est.z,vel_est.t-t0,mean(vel_est.z[:,2:3],2,),"--")
    grid("on")
    legend(["v","v_fl","v_fr","v_bl","v_br","v_mean"])
end