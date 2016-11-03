
using PyPlot
using JLD

function main()
    log_path_record_1 = "$(homedir())/open_loop/output-record-8bbb.jld"
    log_path_record_2 = "$(homedir())/open_loop/output-record-3593.jld"
    d_rec_1 = load(log_path_record)
    d_rec_2 = load(log_path_record)

    imu_meas    = d_rec["imu_meas"]
    gps_meas    = d_rec["gps_meas"]
    cmd_log     = d_rec["cmd_log"]
    cmd_pwm_log = d_rec["cmd_pwm_log"]
    vel_est     = d_rec["vel_est"]
    pos_info    = d_rec["pos_info"]
