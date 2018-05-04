#!/usr/bin/env julia

#=
 Licensing Information: You are free to use or extend these projects for 
 education or reserach purposes provided that (1) you retain this notice
 and (2) you provide clear attribution to UC Berkeley, including a link 
 to http://barc-project.com

 Attibution Information: The barc project ROS code-base was developed
 at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
 (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
 by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
 based on an open source project by Bruce Wootton
=# 

using RobotOS
@rosimport barc.msg: ECU, Vel_est, pos_info, xy_prediction
@rosimport geometry_msgs.msg: Vector3
@rosimport sensor_msgs.msg: Imu
@rosimport marvelmind_nav.msg: hedge_pos
@rosimport std_msgs.msg: Header
rostypegen()
using barc.msg
using geometry_msgs.msg
using sensor_msgs.msg
using std_msgs.msg
using marvelmind_nav.msg
using JLD

include("barc_lib/classes.jl")
include("barc_lib/simModel.jl")

# This type contains measurement data (time, values and a counter)
type Measurements{T}
    i::Int64          # measurement counter
    t::Array{Float64}       # time data
    t_msg::Array{Float64}
    z::Array{T}       # measurement values
end

# This function cleans the zeros from the type above once the simulation is finished
function clean_up(m::Measurements)
    m.t     = m.t[1:m.i-1]
    m.t_msg = m.t_msg[1:m.i-1]
    m.z     = m.z[1:m.i-1,:]
end

function ECU_callback(msg::ECU,u_current::Array{Float64},cmd_log::Measurements)
    u_current[:] = convert(Array{Float64,1},[msg.motor, msg.servo])
    cmd_log.t_msg[cmd_log.i]    = to_sec(get_rostime())
    cmd_log.t[cmd_log.i]        = to_sec(get_rostime())
    cmd_log.z[cmd_log.i,:]      = u_current
    cmd_log.i += 1
end

function input_callback(msg::xy_prediction, input_pred::Array{Float64})
    input_pred[:, 1] = msg.acc
    input_pred[:, 2] = msg.steering
end

function main() 
    u_current = zeros(Float64,2)      # msg ECU is Float32 !
    input_pred = zeros(10, 2)

    buffersize = 60000
    gps_meas = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
    imu_meas = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))
    cmd_log  = Measurements{Float64}(1,ones(buffersize)*Inf,ones(buffersize)*Inf,zeros(buffersize,2))
    z_real   = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,8))
    slip_a   = Measurements{Float64}(1,zeros(buffersize),zeros(buffersize),zeros(buffersize,2))

    # initiate node, set up publisher / subscriber topics
    init_node("barc_sim")
    pub_gps = Publisher("hedge_pos", hedge_pos, queue_size=1)::RobotOS.Publisher{marvelmind_nav.msg.hedge_pos}
    pub_imu = Publisher("imu/data", Imu, queue_size=1)::RobotOS.Publisher{sensor_msgs.msg.Imu}
    pub_vel = Publisher("vel_est", Vel_est, queue_size=1)::RobotOS.Publisher{barc.msg.Vel_est}
    real_val = Publisher("real_val", pos_info, queue_size=1)::RobotOS.Publisher{barc.msg.pos_info}

    s1  = Subscriber("ecu", ECU, ECU_callback, (u_current,cmd_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.ECU}

    # input_pred_sub = Subscriber("xy_prediction", xy_prediction, input_callback, (input_pred, ),
    #                           queue_size=1)::RobotOS.Subscriber{barc.msg.xy_prediction}

    z_current = zeros(60000,8)
    z_current[1,:] = [0.1 0.0 0.0 0.0 0.0 0.0 0.0 0.0]
    # z_current[1, :] = [2.85 (-4.15) 0.0 0.0 0.0 0.0 0.0 0.0]
    slip_ang = zeros(60000,2)

    dt = 0.01
    loop_rate = Rate(1/dt)

    i = 2

    dist_traveled = randn(3)        # encoder positions of three wheels
    last_updated  = 0.0

    r_tire      = 0.036             # radius from tire center to perimeter along magnets [m]
    
    imu_drift = 0.0       # simulates yaw-sensor drift over time (slow sine)

    modelParams     = ModelParams()
    run_id          = get_param("run_id")
    # modelParams.l_A = copy(get_param("L_a"))      # always throws segmentation faults *after* execution!!! ??
    # modelParams.l_B = copy(get_param("L_a"))
    # modelParams.m   = copy(get_param("m"))
    # modelParams.I_z = copy(get_param("I_z"))

    modelParams.l_A = 0.125
    modelParams.l_B = 0.125
    modelParams.m   = 1.98
    modelParams.I_z = 0.03#0.24             # using homogenous distributed mass over a cuboid

    println("Publishing sensor information. Simulator running.")
    imu_data    = Imu()
    vel_est     = Vel_est()
    t0          = to_sec(get_rostime())
    gps_data    = hedge_pos()
    real_data   = pos_info()

    const NOISE = true

    z_real.t_msg[1] = t0
    slip_a.t_msg[1] = t0
    z_real.t[1]     = t0
    slip_a.t[1]     = t0
    t               = 0.0

    sim_gps_interrupt   = 0                 # counter if gps interruption is simulated
    vel_dist_update     = 2*pi*0.036/2      # distance to travel until velocity is updated (half wheel rotation)

    gps_header = Header()
    while ! is_shutdown()
        t_ros   = get_rostime()
        t       = to_sec(t_ros)
        # if sizeof(cmd_log.z[t .> cmd_log.t + 0.2,2]) >= 1
        #    u_current[2] = cmd_log.z[t.>=cmd_log.t+0.2,2][end]       # artificial steering input delay
        # end
        # update current state with a new row vector
        z_current[i,:],slip_ang[i,:]  = simDynModel_exact_xy(z_current[i-1,:], u_current', dt, modelParams)

        #=
        if sumabs(input_pred) > 0.1
            actual_prediction = copy(z_current[i - 1, :])
            println("ACTUAL PREDICTION")
            for i = 1 : 10
                actual_prediction, = simDynModel_exact_xy(actual_prediction, input_pred[i, :], dt, modelParams)
                println(actual_prediction)
            end
        end
        =#

        z_real.t_msg[i] = t
        z_real.t[i]     = t
        slip_a.t_msg[i] = t
        slip_a.t[i]     = t

        # IMU measurements
        if i%2 == 0                 # 50 Hz            
            if NOISE
                imu_drift   = 1+(t-t0)/100#sin(t/100*pi/2)     # drifts to 1 in 100 seconds (and add random start value 1)
                rand_yaw = 0.05*randn()
                if rand_yaw > 0.1
                    rand_yaw = 0.1
                elseif rand_yaw<-0.1
                    rand_yaw=-0.1
                end
            else 
                imu_drift = 0.0
                rand_yaw = 0.0
            end

            yaw         = z_current[i,5] + imu_drift + rand_yaw#+ 0.002*randn() 

            if NOISE
                rand_psiDot = 0.01*randn()
                if rand_psiDot > 0.1
                    rand_psiDot = 0.1
                elseif rand_psiDot<-0.1
                    rand_psiDot=-0.1
                end
            else
                rand_psiDot = 0.0
            end

            psiDot      = z_current[i,6] +rand_psiDot#+ 0.001*randn()
            imu_meas.t_msg[imu_meas.i] = t
            imu_meas.t[imu_meas.i] = t
            imu_meas.z[imu_meas.i,:] = [yaw psiDot]
            imu_meas.i += 1
            imu_data.orientation = geometry_msgs.msg.Quaternion(cos(yaw/2), sin(yaw/2), 0, 0)
            imu_data.angular_velocity = Vector3(0,0,psiDot)
            imu_data.header.stamp = t_ros

            if NOISE
                rand_accX = 0.01*randn()
                if rand_accX > 0.1
                    rand_accX = 0.1
                elseif rand_accX<-0.1
                    rand_accX=-0.1
                end
            else
                rand_accX = 0.0
            end

            imu_data.linear_acceleration.x = diff(z_current[i-1:i,3])[1]/dt - z_current[i,6]*z_current[i,4] #+rand_accX#+ randn()*0.3*1.0

            if NOISE
                rand_accY = 0.01*randn()
                if rand_accY > 0.1
                    rand_accY = 0.1
                elseif rand_accY<-0.1
                    rand_accY=-0.1
                end
            else
                rand_accY = 0.0
            end

            imu_data.linear_acceleration.y = diff(z_current[i-1:i,4])[1]/dt + z_current[i,6]*z_current[i,3] #+rand_accY#+ randn()*0.3*1.0
            publish(pub_imu, imu_data)      # Imu format is defined by ROS, you can look it up by google "rosmsg Imu"
                                            # It's sufficient to only fill the orientation part of the Imu-type (with one quaternion)
        end

        # real values
        if i%2 == 0
            real_data.psiDot = z_current[i,6]
            real_data.psi    = z_current[i,5]
            real_data.v_x    = z_current[i,3]
            real_data.v_y    = z_current[i,4]
            real_data.x      = z_current[i,1]
            real_data.y      = z_current[i,2]
            publish(real_val,real_data)
        end

        # Velocity measurements
        dist_traveled += norm(diff(z_current[i-1:i,1:2]))
        if i%5 == 0                 # 20 Hz
            if sum(dist_traveled .>= vel_dist_update)>=1 && z_current[i,3] > 0.1     # only update if at least one of the magnets has passed the sensor
            # if true
                dist_traveled[dist_traveled.>=vel_dist_update] = 0
                vel_est.vel_est = convert(Float32,norm(z_current[i,3:4]))#+0.00*randn())
                vel_est.vel_fl = convert(Float32,0)
                vel_est.vel_fr = convert(Float32,0)
                vel_est.vel_bl = convert(Float32,0)
                vel_est.vel_br = convert(Float32,0)
            end
            vel_est.header.stamp = t_ros
            publish(pub_vel, vel_est)
        end

        # GPS measurements
        if i%6 == 0               # 16 Hz

            if NOISE
                rand_x = 0.01*randn()
                if rand_x > 0.1
                    rand_x = 0.1
                elseif rand_x<-0.1
                    rand_x=-0.1
                end
            else
                rand_x = 0.0
            end

            x = round(z_current[i,1] +  rand_x,2)#0.002*randn(),2)       # Indoor gps measures, rounded on cm

            if NOISE
                rand_y = 0.01*randn()
                if rand_y > 0.1
                    rand_y = 0.1
                elseif rand_y<-0.1
                    rand_y=-0.1
                end
            else
                rand_y = 0.0
            end

            y = round(z_current[i,2] + rand_y,2)#0.002*randn(),2)

            if NOISE
                if randn()>10            # simulate gps-outlier (probability about 0.13% for randn()>3, 0.62% for randn()>2.5, 2.3% for randn()>2.0 )
                    x += 1#randn()        # add random value to x and y
                    y -= 1#randn()
                    #sim_gps_interrupt = 6       # also do a little interruption
                elseif randn()>10 && sim_gps_interrupt < 0
                    sim_gps_interrupt = 10      # simulate gps-interrupt (10 steps at 25 Hz is 0.4 seconds)
                end
            end

            if sim_gps_interrupt < 0
                gps_meas.t_msg[gps_meas.i] = t
                gps_meas.t[gps_meas.i] = t
                gps_meas.z[gps_meas.i,:] = [x y]
                gps_meas.i += 1
                # gps_data.header.stamp = get_rostime()
                gps_data.x_m = x
                gps_data.y_m = y
                publish(pub_gps, gps_data)
            end
            sim_gps_interrupt -= 1
        end
        i += 1
        rossleep(loop_rate)
    end

    # Clean up buffers
    clean_up(gps_meas)
    clean_up(imu_meas)
    clean_up(cmd_log)
    z_real.z[1:i-1,:] = z_current[1:i-1,:]
    slip_a.z[1:i-1,:] = slip_ang[1:i-1,:]
    z_real.i = i
    slip_a.i = i
    clean_up(z_real)
    clean_up(slip_a)

    # Save simulation data to file
    log_path = "$(homedir())/simulations/output-SIM-$(run_id[1:4]).jld"
    save(log_path,"gps_meas",gps_meas,"z",z_real,"imu_meas",imu_meas,"cmd_log",cmd_log,"slip_a",slip_a)
    println("Exiting node... Saving data to $log_path. Simulated $((i-1)*dt) seconds.")

end

if ! isinteractive()
    main()
end
