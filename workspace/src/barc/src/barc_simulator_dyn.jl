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
@rosimport barc.msg: ECU, Vel_est
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
@rosimport sensor_msgs.msg: Imu
@rosimport marvelmind_nav.msg: hedge_pos
@rosimport std_msgs.msg: Header
rostypegen()
using barc.msg
using data_service.msg
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
    z::Array{T}       # measurement values
end

# This function cleans the zeros from the type above once the simulation is finished
function clean_up(m::Measurements)
    m.t = m.t[1:m.i-1]
    m.z = m.z[1:m.i-1,:]
end

function ECU_callback(msg::ECU,u_current::Array{Float64},cmd_log::Measurements)
    u_current[:] = convert(Array{Float64,1},[msg.motor, msg.servo])
    cmd_log.i += 1
    cmd_log.t[cmd_log.i] = time()
    cmd_log.z[cmd_log.i,:] = u_current
end

function main() 
    u_current = zeros(Float64,2)      # msg ECU is Float32 !

    buffersize = 60000
    gps_meas = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))
    imu_meas = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))
    cmd_log  = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))
    z_real   = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,8))
    slip_a   = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))

    z_real.t[1]   = time()
    slip_a.t[1]   = time()
    imu_meas.t[1] = time()
    cmd_log.t[1]  = time()

    # initiate node, set up publisher / subscriber topics
    init_node("barc_sim")
    pub_gps = Publisher("hedge_pos", hedge_pos, queue_size=1)::RobotOS.Publisher{marvelmind_nav.msg.hedge_pos}
    pub_imu = Publisher("imu/data", Imu, queue_size=1)::RobotOS.Publisher{sensor_msgs.msg.Imu}
    pub_vel = Publisher("vel_est", Vel_est, queue_size=1)::RobotOS.Publisher{barc.msg.Vel_est}

    s1  = Subscriber("ecu", ECU, ECU_callback, (u_current,cmd_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.ECU}

    z_current = zeros(60000,8)
    z_current[1,:] = [0.1 0.0 0.0 0.0 0.0 0.0 0.0 0.0]
    slip_ang = zeros(60000,2)

    dt = 0.01
    loop_rate = Rate(1/dt)

    i = 2

    dist_traveled = 0.0
    last_updated  = 0.0

    r_tire      = 0.036                  # radius from tire center to perimeter along magnets [m]
    
    imu_drift = 0.0       # simulates yaw-sensor drift over time (slow sine)

    modelParams     = ModelParams()
    run_id          = get_param("run_id")
    # modelParams.l_A = copy(get_param("L_a"))      # always throws segmentation faults *after* execution!!! ??
    # modelParams.l_B = copy(get_param("L_a"))
    # modelParams.m   = copy(get_param("m"))
    # modelParams.I_z = copy(get_param("I_z"))

    modelParams.l_A = 0.125
    modelParams.l_B = 0.125
    modelParams.m = 1.98
    modelParams.I_z = 0.24

    println("Publishing sensor information. Simulator running.")
    imu_data    = Imu()
    vel_est     = Vel_est()
    t0          = time()
    gps_data    = hedge_pos()
    
    t           = 0.0

    sim_gps_interrupt   = 0                 # counter if gps interruption is simulated
    vel_pos             = zeros(2)          # position when velocity was updated last time
    vel_dist_update     = 2*pi*0.036/2      # distance to travel until velocity is updated (half wheel rotation)

    gps_header = Header()
    while ! is_shutdown()

        t = time()
        t_ros = get_rostime()
        # update current state with a new row vector
        z_current[i,:],slip_ang[i,:]  = simDynModel_exact_xy(z_current[i-1,:],u_current', dt, modelParams)
        z_real.t[i]     = t
        slip_a.t[i]     = t

        # IMU measurements
        if i%2 == 0                 # 50 Hz
            imu_drift   = 1+(t-t0)/100#sin(t/100*pi/2)     # drifts to 1 in 100 seconds (and add random start value 1)
            yaw         = z_current[i,5] + randn()*0.02 + imu_drift
            psiDot      = z_current[i,6] + 0.01*randn()
            imu_meas.i += 1
            imu_meas.t[imu_meas.i] = t
            imu_meas.z[imu_meas.i,:] = [yaw psiDot]
            imu_data.orientation = geometry_msgs.msg.Quaternion(cos(yaw/2), sin(yaw/2), 0, 0)
            imu_data.angular_velocity = Vector3(0,0,psiDot)
            imu_data.header.stamp = t_ros
            publish(pub_imu, imu_data)      # Imu format is defined by ROS, you can look it up by google "rosmsg Imu"
                                            # It's sufficient to only fill the orientation part of the Imu-type (with one quaternion)
        end

        # Velocity measurements
        if i%5 == 0                 # 20 Hz
            if norm(z_current[i,1:2][:]-vel_pos) > vel_dist_update     # only update if a magnet has passed the sensor
                vel_est.vel_est = convert(Float32,norm(z_current[i,3:4])+0.01*randn())
                vel_pos = z_current[i,1:2][:]
            end
            vel_est.header.stamp = t_ros
            publish(pub_vel, vel_est)
        end

        # GPS measurements
        if i%4 == 0               # 25 Hz
            x = round(z_current[i,1] + 0.02*randn(),2)       # Indoor gps measures, rounded on cm
            y = round(z_current[i,2] + 0.02*randn(),2)
            if randn()>3            # simulate gps-outlier (probability about 0.13% for randn()>3, 0.62% for randn()>2.5, 2.3% for randn()>2.0 )
                x += 1#randn()        # add random value to x and y
                y -= 1#randn()
                #sim_gps_interrupt = 6       # also do a little interruption
            elseif randn()>3 && sim_gps_interrupt < 0
                sim_gps_interrupt = 10      # simulate gps-interrupt (10 steps at 25 Hz is 0.4 seconds)
            end
            if sim_gps_interrupt < 0
                gps_meas.i += 1
                gps_meas.t[gps_meas.i] = t
                gps_meas.z[gps_meas.i,:] = [x y]
                gps_data.header.stamp = get_rostime()
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
    #writedlm(log_path,z_current[1:i-1,:])
end

if ! isinteractive()
    main()
end
