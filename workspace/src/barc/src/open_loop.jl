#!/usr/bin/env julia


##########################################################################################################################################################################
##########################################################################################################################################################################

# using RobotOS
# @rosimport barc.msg: ECU, pos_info, Vel_est
# @rosimport geometry_msgs.msg: Vector3
# @rosimport sensor_msgs.msg: Imu
# @rosimport marvelmind_nav.msg: hedge_pos
# rostypegen()
# using barc.msg
# using geometry_msgs.msg
# using sensor_msgs.msg
# using std_msgs.msg
# using marvelmind_nav.msg
# using JLD
# include("barc_lib/log_functions.jl")

# function main()

#     # Set up logging
#     buffersize      = 60000
#     gps_meas        = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
#     imu_meas        = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,9))
#     cmd_log         = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
#     cmd_pwm_log     = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
#     vel_est_log     = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,5))
#     pos_info_log    = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,20))

#     # Initialize ROS node and topics
#     init_node("mpc_traj")
#     s1  = Subscriber("ecu", ECU, ECU_callback, (cmd_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.ECU}
#     s2  = Subscriber("imu/data", Imu, IMU_callback, (imu_meas,), queue_size=1)::RobotOS.Subscriber{sensor_msgs.msg.Imu}
#     s3  = Subscriber("ecu_pwm", ECU, ECU_PWM_callback, (cmd_pwm_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.ECU}
#     s4  = Subscriber("hedge_pos", hedge_pos, GPS_callback, (gps_meas,), queue_size=1)::RobotOS.Subscriber{marvelmind_nav.msg.hedge_pos}
#     s5  = Subscriber("pos_info", pos_info, pos_info_callback, (pos_info_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
#     s6  = Subscriber("vel_est", Vel_est, vel_est_callback, (vel_est_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.Vel_est}

#     run_id      = get_param("run_id")

#     loop_rate   = Rate(10)
#     pub         = Publisher("ecu_pwm", ECU, queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
#     cmd         = ECU()      # command type
#     cmd.motor   = 0
#     cmd.servo   = 0
#     cmd.header.stamp = get_rostime()
#     println("Starting open loop control. Waiting.")
#     rossleep(3.0)

#     t0_ros      = get_rostime()
#     t0          = to_sec(t0_ros)
#     t           = 0.0

#     gps_meas.i      = 1
#     imu_meas.i      = 1
#     cmd_log.i       = 1
#     cmd_pwm_log.i   = 1
#     vel_est_log.i   = 1
#     pos_info_log.i  = 1
    
#     chicane_speed = 97#1.0
#     chicane_turn  = [120,60]

#     cmd_m = 94
#     cmd_s = 80
#     t_next = 7
#     mode = 1        # 1 = acc, 2 = break, 3 = break
#     # Idea: 5 seconds acc, 3 seconds breaking.

#     # Start node
#     while ! is_shutdown()#t < 29.0              # run exactly x seconds
#         t = to_sec(get_rostime())-t0
#         if t <= 3
#             cmd.motor = 90.0
#             cmd.servo = 80.0
#         elseif t <= t_next
#             if mode == 1
#                 cmd.motor = cmd_m
#                 cmd.servo = cmd_s
#             elseif mode == 2
#                 cmd.motor = 70
#             end
#         elseif t <= 300
#             if mode == 1
#                 mode = 2
#             else
#                 cmd_s += 0
#                 cmd_m += 1
#                 mode = 1
#             end
#             t_next = t + 4
#         # **************************************
#         # CHICANE:
#         # elseif t <= 3
#         #     cmd.motor = chicane_speed
#         #     cmd.servo = chicane_turn[1]
#         # elseif t <= 4
#         #     cmd.motor = chicane_speed
#         #     cmd.servo = chicane_turn[2]
#         # elseif t <= 5
#         #     cmd.motor = chicane_speed
#         #     cmd.servo = chicane_turn[1]
#         # elseif t <= 6
#         #     cmd.motor = chicane_speed
#         #     cmd.servo = chicane_turn[2]
#         # elseif t <= 7
#         #     cmd.motor = chicane_speed
#         #     cmd.servo = chicane_turn[1]
#         # elseif t <= 8
#         #     cmd.motor = chicane_speed
#         #     cmd.servo = chicane_turn[2]
#         # elseif t <= 9
#         #     cmd.motor = chicane_speed
#         #     cmd.servo = chicane_turn[1]
#         # elseif t <= 10
#         #     cmd.motor = chicane_speed
#         #     cmd.servo = chicane_turn[2]
#         # elseif t <= 11
#         #     cmd.motor = chicane_speed
#         #     cmd.servo = chicane_turn[1]
#         # **************************************
#         # CONTINUOUS ACCELERATION:
#         # elseif t <= 123
#         #     cmd.motor = 0.0+(t-3)/20
#         #     cmd.servo = 0.0#-(t-3.0)/300-0.15
#         # elseif t <= 8                   # CHECK TIME AND ACCELERATION !!!
#         #     cmd.motor = 97.0             # CHECK TIME AND ACCELERATION !!!
#         #     cmd.servo = 100.0
#         # elseif t <= 13                   # CHECK TIME AND ACCELERATION !!!
#         #     cmd.motor = 97.0             # CHECK TIME AND ACCELERATION !!!
#         #     cmd.servo = 105.0
#         # elseif t <= 18                   # CHECK TIME AND ACCELERATION !!!
#         #     cmd.motor = 97.0             # CHECK TIME AND ACCELERATION !!!
#         #     cmd.servo = 110.0
#         # elseif t <= 23                   # CHECK TIME AND ACCELERATION !!!
#         #     cmd.motor = 97.0             # CHECK TIME AND ACCELERATION !!!
#         #     cmd.servo = 115.0
#         # elseif t <= 28                   # CHECK TIME AND ACCELERATION !!!
#         #     cmd.motor = 97.0             # CHECK TIME AND ACCELERATION !!!
#         #     cmd.servo = 120.0
#         # elseif t <= 33                   # CHECK TIME AND ACCELERATION !!!
#         #     cmd.motor = 1.0             # CHECK TIME AND ACCELERATION !!!
#         #     cmd.servo = -0.4
#         # elseif t <= 38                   # CHECK TIME AND ACCELERATION !!!
#         #     cmd.motor = 1.0             # CHECK TIME AND ACCELERATION !!!
#         #     cmd.servo = -0.45
#         # elseif t <= 43                   # CHECK TIME AND ACCELERATION !!!
#         #     cmd.motor = 1.0             # CHECK TIME AND ACCELERATION !!!
#         #     cmd.servo = -0.5
#         # elseif t <= 44
#         #     cmd.motor = -2.0
#         #     cmd.servo = 0
#         else
#             cmd.motor = 70.0
#             cmd.servo = 90.0
#         end
#         cmd.header.stamp = get_rostime()
#         publish(pub, cmd)  

#         rossleep(loop_rate)
#     end

#     # Clean up buffers
#     clean_up(gps_meas)
#     clean_up(imu_meas)
#     clean_up(cmd_log)
#     clean_up(cmd_pwm_log)
#     clean_up(pos_info_log)
#     clean_up(vel_est_log)

#     println("Exiting open loop control.")
#     log_path = "$(homedir())/open_loop/output-record-$(run_id[1:4]).jld"
#     if isfile(log_path)
#         log_path = "$(homedir())/open_loop/output-record-$(run_id[1:4])-2.jld"
#         warn("Warning: File already exists.")
#     end
#     save(log_path,"gps_meas",gps_meas,"imu_meas",imu_meas,"cmd_log",cmd_log,"cmd_pwm_log",cmd_pwm_log,"pos_info",pos_info_log,"vel_est",vel_est_log)
#     println("Exiting node... Saved recorded data to $log_path.")
# end

# if ! isinteractive()
#     main()
# end


######################################################################################################################################################################
######################################################################################################################################################################
######################################################################################################################################################################



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
using PyPlot
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

    t_next = 2 # SET THIS TO 4 IF YOU DO ACCELERATION TESTS

    gps_meas.i      = 1
    imu_meas.i      = 1
    cmd_log.i       = 1
    cmd_pwm_log.i   = 1
    vel_est_log.i   = 1
    pos_info_log.i  = 1

    cmd_m = 100 # PWM COMMAND FOR MOTOR

    cmd_s = 93  # PWM COMMAND FOR STEERING

    # t_cmd = zeros(10)
    # t_no_cmd = zeros(10)

    # index=1

    # time_index = 0

    # a=zeros(10)


    while ! is_shutdown()             # run exactly x seconds
        t = to_sec(get_rostime())-t0




        if t <= t_next 

            cmd.motor = cmd_m
            cmd.servo = cmd_s

            #t_cmd[index] = t
            
        elseif t > t_next && t <= t_next + 4 

            cmd.motor = 40

            #t_no_cmd[index] = t


        else
            cmd.motor = -1

            println("******FLAG*******")
        #     t_next = t + 4

        #     println("t== ",t)
        #     println("time_index== ",time_index)

        #     time_vector = time_index+0.1:.02:t-0.1

        #     v = zeros(length(time_vector))
        #     command = zeros(length(time_vector))

        #      println("v= ",v)

        #     # println("time vector= ",time_vector)

        #     for i=1:length(time_vector)
                
        #         v[i] = vel_est_log.z[time_vector[i].>vel_est_log.t,1][end]
        #         command[i] = cmd_pwm_log.z[time_vector[i].>cmd_pwm_log.t,1][end]
        #     end

        #     println("v= ",v)

        #     v_opt   = v[1:end]
        #     command_opt = command[1:end]
        #     v_opt   = v[command.>90]
        #     command_opt = command[command.>90]

        #     figure()
        #     plot(v_opt)

        #     # println("FLAG=",v_opt)

        #     a[index] = v_opt[end]/4

        #     println("a= ",a[index])



        #     index=index+1

        #     time_index = t
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
    save(log_path,"gps_meas",gps_meas,"imu_meas",imu_meas,"cmd_log",cmd_log,"cmd_pwm_log",cmd_pwm_log,"pos_info",pos_info_log,"vel_est",vel_est_log)#,"t_cmd",t_cmd,"t_no_cmd",t_no_cmd)
    println("Exiting node... Saved recorded data to $log_path.")
end

if ! isinteractive()
    main()
end

       





