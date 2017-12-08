#!/usr/bin/env julia

#=
This node listens to and records messages of following topics:
Raw GPS (/hedge_pos)
Raw IMU (/imu/data)
Commands (/ecu)
Raw velocity estimation (/vel_est)
Position info (/pos_info)
=#

using RobotOS
@rosimport barc.msg: ECU, pos_info, Vel_est
#@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
@rosimport sensor_msgs.msg: Imu
@rosimport marvelmind_nav.msg: hedge_pos
rostypegen()
using barc.msg
#using data_service.msg
using geometry_msgs.msg
using sensor_msgs.msg
using std_msgs.msg
using marvelmind_nav.msg
using JLD

include("barc_lib/log_functions.jl")

function main() 

    buffersize      = 60000
    gps_meas        = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
    imu_meas        = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,9))
    cmd_log         = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
    cmd_pwm_log     = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
    vel_est_log     = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,5))
    pos_info_log    = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,20))

    # initiate node, set up publisher / subscriber topics
    init_node("barc_record")
    s1  = Subscriber("ecu", ECU, ECU_callback, (cmd_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.ECU}
    s2  = Subscriber("imu/data", Imu, IMU_callback, (imu_meas,), queue_size=1)::RobotOS.Subscriber{sensor_msgs.msg.Imu}
    s3  = Subscriber("ecu_pwm", ECU, ECU_PWM_callback, (cmd_pwm_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.ECU}
    s4  = Subscriber("hedge_pos", hedge_pos, GPS_callback, (gps_meas,), queue_size=30)::RobotOS.Subscriber{marvelmind_nav.msg.hedge_pos}
    s5  = Subscriber("pos_info", pos_info, pos_info_callback, (pos_info_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
    s6  = Subscriber("vel_est", Vel_est, vel_est_callback, (vel_est_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.Vel_est}

    run_id = get_param("run_id")

    println("Recorder running.")
    spin()                              # wait for callbacks until shutdown

    # Clean up buffers
    clean_up(gps_meas)
    clean_up(imu_meas)
    clean_up(cmd_log)
    clean_up(cmd_pwm_log)
    clean_up(pos_info_log)
    clean_up(vel_est_log)

    # Save simulation data to file
    #log_path = "$(homedir())/simulations/record-$(Dates.format(now(),"yyyy-mm-dd-HH-MM-SS")).jld"
    log_path = "$(homedir())/simulations/output-record-$(run_id[1:4]).jld"
    if isfile(log_path)
        log_path = "$(homedir())/simulations/output-record-$(run_id[1:4])-2.jld"
        warn("Warning: File already exists.")
    end
    save(log_path,"gps_meas",gps_meas,"imu_meas",imu_meas,"cmd_log",cmd_log,"cmd_pwm_log",cmd_pwm_log,"pos_info",pos_info_log,"vel_est",vel_est_log)
    println("Exiting node... Saving recorded data to $log_path.")
end

if ! isinteractive()
    main()
end
