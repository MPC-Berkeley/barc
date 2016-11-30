
using PyPlot
using JLD
using Optim

# type Measurements{T}
#     i::Int64                # measurement counter
#     t::Array{Float64}       # time data (when it was received by this recorder)
#     t_msg::Array{Float64}   # time that the message was sent
#     z::Array{T}             # measurement values
# end

function main(code::AbstractString)
    #log_path_record = ("$(homedir())/open_loop/output-record-8bbb.jld","$(homedir())/open_loop/output-record-3593.jld")
    log_path_record = "$(homedir())/open_loop/output-record-$code.jld"
    d_rec = load(log_path_record)

    cmd_pwm_log = d_rec["cmd_pwm_log"]
    vel_est     = d_rec["vel_est"]
    pos_info    = d_rec["pos_info"]

    t0 = max(cmd_pwm_log.t[1],vel_est.t[1])
    t_end = min(cmd_pwm_log.t[end],vel_est.t[end])

    t = t0+0.1:.02:t_end-0.1
    v = zeros(length(t))
    cmd = zeros(length(t))

    for i=1:length(t)
        #v[i] = vel_est.z[t[i].>vel_est.t,1][end]
        v[i] = pos_info.z[t[i].>pos_info.t,15][end]
        cmd[i] = cmd_pwm_log.z[t[i].>cmd_pwm_log.t,1][end]
    end
    v_opt   = v[1:end]
    cmd_opt = cmd[1:end]
    v_opt   = v[cmd.>94]
    cmd_opt = cmd[cmd.>94]

    res = optimize(c->cost(cmd_opt,v_opt,c,0),[0.001,0,0.1,0.001,0])
    c = Optim.minimizer(res)
    c2 = [1/c[1], c[2]/c[1], 1/c[4], c[5]/c[4]]
    cost(cmd_opt,v_opt,c,1)
    println("c = $c")
    println("and c_2 = $c2")
end

function v_over_u(code::AbstractString)
    log_path_record = "$(homedir())/open_loop/output-record-$code.jld"
    d_rec = load(log_path_record)

    cmd_pwm_log = d_rec["cmd_pwm_log"]
    vel_est     = d_rec["vel_est"]
    pos_info    = d_rec["pos_info"]
    imu_meas    = d_rec["imu_meas"]

    t0 = max(cmd_pwm_log.t[1],vel_est.t[1])
    t_end = min(cmd_pwm_log.t[end],vel_est.t[end])

    L_b = 0.125
    t = t0+0.1:.02:t_end-0.1
    v = zeros(length(t))
    cmd = zeros(length(t),2)
    psiDot = zeros(length(t))

    for i=1:length(t)
        #v[i] = vel_est.z[t[i].>vel_est.t,1][end]
        v[i] = pos_info.z[t[i].>pos_info.t,15][end]
        cmd[i,:] = cmd_pwm_log.z[t[i].>cmd_pwm_log.t,:][end,:]
        psiDot[i] = imu_meas.z[t[i].>imu_meas.t,3][end]
    end
    v_x = real(sqrt(complex(v.^2-psiDot.^2*L_b^2)))
    v_y = L_b*psiDot
    delta = atan2(psiDot*0.25,v_x)
    c = ((cmd[:,1]-91)/6.5-0.5*v)./delta.^2
    c = c[abs(c).<100]
    plot(c)
    grid("on")
    figure()
    plot(v)
    grid("on")
    #plot(cmd[:,1],v)
end

function cost(cmd,v,c,p)
    cost = 0
    v_s = zeros(length(v))
    v_s[1] = v[1]
    v_s[2] = v[2]
    for i=2:length(cmd)-1
        if cmd[i]-cmd[i-1] != 0
            v_s[i] = v[i]
        end
        if cmd[i] > 90
            v_s[i+1] = v_s[i] + 0.02*(c[1]*cmd[i] + c[2] - c[3]*v_s[i])
        else
            v_s[i+1] = max(v_s[i] + 0.02*(c[4]*cmd[i] + c[5] - c[3]*v_s[i]),0)
        end
    end
    if p==1
        plot((1:length(v_s))*0.02,v_s,(1:length(v))*0.02,v)
        grid("on")
        legend(["v_sim","v_meas"])
        title("Curve fitting acceleration")
        xlabel("t [s]")
        ylabel("v [m/s]")
    end
    cost = norm(v_s-v)
    return cost
end
