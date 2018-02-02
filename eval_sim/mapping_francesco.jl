
using PyPlot
using JLD
#using Optim

# type Measurements{T}
#     i::Int64                # measurement counter
#     t::Array{Float64}       # time data (when it was received by this recorder)
#     t_msg::Array{Float64}   # time that the message was sent
#     z::Array{T}             # measurement values
# end

function data_analysis(code::AbstractString)

	log_path_record = "$(homedir())/open_loop/output-record-$code.jld"
    d_rec = load(log_path_record)

    cmd_pwm_log = d_rec["cmd_pwm_log"]
    vel_est     = d_rec["vel_est"]
    # t_cmd       = d_rec["t_cmd"]
    # t_no_cmd    = d_rec["t_no_cmd"]

    t0 = max(cmd_pwm_log.t[1],vel_est.t[1])
    t_end = min(cmd_pwm_log.t[end],vel_est.t[end])

    t = t0+0.1:.02:t_end-0.1
    v = zeros(length(t))
    cmd = zeros(length(t))

    for i=1:length(t)
        v[i] = vel_est.z[t[i].>vel_est.t,1][end]
        #v[i] = pos_info.z[t[i].>pos_info.t,15][end]
        cmd[i] = cmd_pwm_log.z[t[i].>cmd_pwm_log.t,1][end]
    end

    v_opt   = v[1:end]
    cmd_opt = cmd[1:end]
    v_opt   = v[cmd.>90]
    cmd_opt = cmd[cmd.>90]

    figure()
    plot(v_opt)

    # half = round(length(v_opt)/2)
    # println(half)

    #v_mean = mean(v_opt[half:end])

    #v_mean = mean(v_opt[length(v_opt)/2:end])

    #a = v_mean/4

    a = v_opt[end]/4

    println("a= ",a)

    figure(2)

    plot(cmd_opt[1],a, "*")


    # println(t_cmd)
    # println(t_no_cmd)
    # println(t0)

    # figure()
    # plot(cmd,v)

    # figure()
    # plot(t,cmd)

    # figure()
    # plot(t,v)

    # figure()
    # plot(v)

    # figure() 
    # plot(t)
end