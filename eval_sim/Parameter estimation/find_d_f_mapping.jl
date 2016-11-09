
using PyPlot
using JLD

type Measurements{T}
    i::Int64                # measurement counter
    t::Array{Float64}       # time data (when it was received by this recorder)
    t_msg::Array{Float64}   # time that the message was sent
    z::Array{T}             # measurement values
end

function main()
    #log_path_record = ("$(homedir())/open_loop/output-record-8bbb.jld","$(homedir())/open_loop/output-record-3593.jld")
    log_path_record = ("$(homedir())/open_loop/output-record-1486.jld",)
    res_cmd = Float64[]
    res_delta = Float64[]
    res_t = Float64[]
    L_b = 0.125
    for j=1:1
        d_rec = load(log_path_record[j])

        imu_meas    = d_rec["imu_meas"]
        cmd_pwm_log = d_rec["cmd_pwm_log"]
        vel_est     = d_rec["vel_est"]

        t0 = max(cmd_pwm_log.t[1],vel_est.t[1],imu_meas.t[1])+0.5
        t_end = min(cmd_pwm_log.t[end],vel_est.t[end],imu_meas.t[end])-0.5

        t = t0+0.1:.02:t_end-0.1
        v = zeros(length(t))
        psiDot = zeros(length(t))
        cmd = zeros(length(t))

        for i=1:length(t)
            if cmd_pwm_log.z[t[i].>cmd_pwm_log.t,2][end] < 90 || cmd_pwm_log.z[t[i].>cmd_pwm_log.t,2][end] > 95
                v[i] = vel_est.z[t[i].>vel_est.t,1][end]
                psiDot[i] = imu_meas.z[t[i].>imu_meas.t,3][end]
                cmd[i] = cmd_pwm_log.z[t[i].>cmd_pwm_log.t,2][end]
            end
        end

        v_x = real(sqrt(complex(v.^2-psiDot.^2*L_b^2)))
        res_cmd = cat(1,res_cmd,cmd)
        res_delta = cat(1,res_delta,atan2(psiDot*0.25,v_x))
        res_t = cat(1,t)
    end
    res_delta = res_delta[res_cmd.>0]
    res_cmd = res_cmd[res_cmd.>0]
    res_delta = res_delta[res_cmd.<120]
    res_cmd = res_cmd[res_cmd.<120]
    res_delta = res_delta[res_cmd.>60]
    res_cmd = res_cmd[res_cmd.>60]
    sz = size(res_cmd,1)

    cmd_left = res_cmd[res_cmd.>103]
    delta_left = res_delta[res_cmd.>103]
    cmd_right = res_cmd[res_cmd.<80]
    delta_right = res_delta[res_cmd.<80]

    coeff = [res_cmd ones(sz,1)]\res_delta
    x = [1,200]
    y = [1 1;200 1]*coeff

    coeff_r = [cmd_right ones(size(cmd_right,1),1)]\delta_right
    xr = [1,200]
    yr = [1 1;200 1]*coeff_r

    coeff_l = [cmd_left ones(size(cmd_left,1),1)]\delta_left
    xl = [1,200]
    yl = [1 1;200 1]*coeff_l

    ax1=subplot(211)
    plot(res_t,res_delta)
    grid("on")
    subplot(212,sharex=ax1)
    plot(res_t,res_cmd)
    grid("on")
    
    figure(2)
    plot(res_cmd,res_delta,"*",xl,yl,xr,yr)

    plot(vel_est.t,vel_est.z)
    grid("on")
    legend(["vel_avg","v_fl","v_fr","v_bl","v_br"])
end

function main_pwm(code::AbstractString)
    log_path_record = "$(homedir())/open_loop/output-record-$(code).jld"

    L_b = 0.125
    d_rec = load(log_path_record)

    imu_meas    = d_rec["imu_meas"]
    cmd_pwm_log = d_rec["cmd_pwm_log"]
    vel_est     = d_rec["vel_est"]

    t0 = max(cmd_pwm_log.t[1],vel_est.t[1],imu_meas.t[1])+1.0
    t_end = min(cmd_pwm_log.t[end],vel_est.t[end],imu_meas.t[end])-1.0

    t = t0+0.1:.02:t_end-0.1
    v = zeros(length(t))
    psiDot = zeros(length(t))
    cmd = zeros(length(t))

    for i=1:length(t)
        if cmd_pwm_log.z[t[i].>cmd_pwm_log.t,2][end] < 90 || cmd_pwm_log.z[t[i].>cmd_pwm_log.t,2][end] > 95
            if cmd_pwm_log.z[t[i].>cmd_pwm_log.t,2][end] == cmd_pwm_log.z[t[i]-1.0.>cmd_pwm_log.t,2][end]
            v[i] = vel_est.z[t[i].>vel_est.t,1][end]
            psiDot[i] = imu_meas.z[t[i].>imu_meas.t,3][end]
            cmd[i] = cmd_pwm_log.z[t[i].>cmd_pwm_log.t,2][end]
        end
        end
    end

    v_x = real(sqrt(complex(v.^2-psiDot.^2*L_b^2)))
    delta = atan2(psiDot*0.25,v_x)

    idx = copy( (delta.<1.0) & (delta.> -1.0) & (cmd .< 90) & (cmd .> 50) )
    delta = delta[idx]
    cmd = cmd[idx]

    sz = size(cmd,1)
    coeff = [cmd ones(sz,1)]\delta
    x = [1,200]
    y = [1 1;200 1]*coeff

    plot(cmd,delta,"*",x,y)
    grid("on")

    println("c1 = $(1/coeff[1])")
    println("c2 = $(coeff[2]/coeff[1])")

end