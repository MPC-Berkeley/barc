#!/usr/bin/env julia

using RobotOS
@rosimport barc.msg: ECU, pos_info, Vel_est
@rosimport geometry_msgs.msg: Vector3
@rosimport sensor_msgs.msg: Imu
@rosimport marvelmind_nav.msg: hedge_pos
rostypegen()
using barc.msg
using geometry_msgs.msg
using sensor_msgs.msg
using std_msgs.msg
using marvelmind_nav.msg
using JLD
include("barc_lib/log_functions.jl")

function main()

	# Set up logging
    buffersize      = 60000
    gps_meas        = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
    imu_meas        = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,9))
    cmd_log         = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
    cmd_pwm_log     = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
    vel_est_log     = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,5))
    pos_info_log    = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,20))

    # Initialize ROS node and topics
    init_node("mpc_traj")
    s1  = Subscriber("ecu", ECU, ECU_callback, (cmd_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.ECU}
    s2  = Subscriber("imu/data", Imu, IMU_callback, (imu_meas,), queue_size=1)::RobotOS.Subscriber{sensor_msgs.msg.Imu}
    s3  = Subscriber("ecu_pwm", ECU, ECU_PWM_callback, (cmd_pwm_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.ECU}
    s4  = Subscriber("hedge_pos", hedge_pos, GPS_callback, (gps_meas,), queue_size=1)::RobotOS.Subscriber{marvelmind_nav.msg.hedge_pos}
    s5  = Subscriber("pos_info", pos_info, pos_info_callback, (pos_info_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
    s6  = Subscriber("vel_est", Vel_est, vel_est_callback, (vel_est_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.Vel_est}

    run_id      = get_param("run_id")

    loop_rate   = Rate(10)
    pub         = Publisher("ecu_pwm", ECU, queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
    cmd         = ECU()      # command type
    cmd.motor   = 0
    cmd.servo   = 0
    cmd.header.stamp = get_rostime()
    println("Starting open loop control. Waiting.")
    rossleep(3.0)

    t0_ros      = get_rostime()
    t0          = to_sec(t0_ros)
    t           = 0.0

    t_next = 4

    gps_meas.i      = 1
    imu_meas.i      = 1
    cmd_log.i       = 1
    cmd_pwm_log.i   = 1
    vel_est_log.i   = 1
    pos_info_log.i  = 1

    cmd_m = 94


    while ! is_shutdown()             # run exactly x seconds
        t = to_sec(get_rostime())-t0

        println(to_sec(get_rostime()))

        t1 = 0
        t2 = 0

        if t <= t_next 

        	cmd.motor = cmd_m
        	
        elseif t > t_next && t <= t_next + 4 

        	cmd.motor = 90

        else
        	t_next = t + 4
        end

        cmd.header.stamp = get_rostime()
        publish(pub, cmd)  

        rossleep(loop_rate)
    end

     # Clean up buffers
    clean_up(gps_meas)
    clean_up(imu_meas)
    clean_up(cmd_log)
    clean_up(cmd_pwm_log)
    clean_up(pos_info_log)
    clean_up(vel_est_log)

    println("Exiting open loop control.")
    log_path = "$(homedir())/open_loop/output-record-$(run_id[1:4]).jld"
    if isfile(log_path)
        log_path = "$(homedir())/open_loop/output-record-$(run_id[1:4])-2.jld"
        warn("Warning: File already exists.")
    end
    save(log_path,"gps_meas",gps_meas,"imu_meas",imu_meas,"cmd_log",cmd_log,"cmd_pwm_log",cmd_pwm_log,"pos_info",pos_info_log,"vel_est",vel_est_log)
    println("Exiting node... Saved recorded data to $log_path.")
end

if ! isinteractive()
    main()
end

       





