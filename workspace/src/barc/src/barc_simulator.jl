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
@rosimport barc.msg: ECU, pos_info, Encoder, Ultrasound, Z_KinBkMdl, Logging, Z_DynBkMdl
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
@rosimport sensor_msgs.msg: Imu
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using sensor_msgs.msg
using JLD

u_current = zeros(2,1)

t = 0


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

buffersize = 60000
gps_meas = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))
imu_meas = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))
est_meas = Measurements{Float32}(0,zeros(buffersize),zeros(Float32,buffersize,4))
est_meas_dyn = Measurements{Float32}(0,zeros(buffersize),zeros(Float32,buffersize,6))
cmd_log  = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))
z_real   = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,4))

z_real.t[1]   = time()
imu_meas.t[1] = time()
est_meas.t[1] = time()
est_meas_dyn.t[1] = time()
cmd_log.t[1]  = time()

function simModel(z,u,dt,l_A,l_B)

   # kinematic bicycle model
   # u[1] = acceleration
   # u[2] = steering angle

    bta = atan(l_A/(l_A+l_B)*tan(u[2]))

    zNext = z
    zNext[1] = z[1] + dt*(z[4]*cos(z[3] + bta))       # x
    zNext[2] = z[2] + dt*(z[4]*sin(z[3] + bta))     # y
    zNext[3] = z[3] + dt*(z[4]/l_B*sin(bta))        # psi
    zNext[4] = z[4] + dt*(u[1] - 0.63 * z[4]^2 * sign(z[4]))                     # v

    # Add process noise (depending on velocity)
    zNext = zNext + diagm([0.001*z[4],0.001*z[4],0.00,0.001*z[4]])*randn(4,1)

    return zNext
end


function ECU_callback(msg::ECU)
    global u_current
    u_current = [msg.motor, msg.servo]
    cmd_log.i += 1
    cmd_log.t[cmd_log.i] = time()
    cmd_log.z[cmd_log.i,:] = u_current'
end

function est_callback(msg::Z_KinBkMdl)
    global est_meas
    est_meas.i += 1
    est_meas.t[est_meas.i]      = time()
    est_meas.z[est_meas.i,:]    = [msg.x msg.y msg.psi msg.v]
end

function est_dyn_callback(msg::Z_DynBkMdl)
    global est_meas_dyn
    est_meas_dyn.i += 1
    est_meas_dyn.t[est_meas_dyn.i]      = time()
    est_meas_dyn.z[est_meas_dyn.i,:]    = [msg.x msg.y msg.v_x msg.v_y msg.psi msg.psi_dot]
end

function main() 
    # initiate node, set up publisher / subscriber topics
    init_node("barc_sim")
    pub_enc = Publisher("encoder", Encoder, queue_size=10)
    pub_gps = Publisher("indoor_gps", Vector3, queue_size=10)
    pub_imu = Publisher("imu/data", Imu, queue_size=10)


    # read the axle distances from the launch file
    l_A = get_param("L_a")       # distance from CoG to front axel
    l_B = get_param("L_b")       # distance from CoG to rear axel

    s1  = Subscriber("ecu", ECU, ECU_callback, queue_size=10)
    s2  = Subscriber("state_estimate", Z_KinBkMdl, est_callback, queue_size=10)
    s3  = Subscriber("state_estimate_dynamic", Z_DynBkMdl, est_dyn_callback, queue_size=10)

    z_current = zeros(60000,4)
    z_current[1,:] = [0.2 0.0 0.0 0.0]

    dt = 0.01
    loop_rate = Rate(1/dt)

    i = 2

    dist_traveled = 0
    last_updated  = 0

    r_tire      = 0.036                  # radius from tire center to perimeter along magnets [m]
    quarterCirc = 0.5 * pi * r_tire      # length of a quarter of a tire, distance from one to the next encoder
    
    FL = 0 #front left wheel encoder counter
    FR = 0 #front right wheel encoder counter
    BL = 0 #back left wheel encoder counter
    BR = 0 #back right wheel encoder counter

    imu_drift = 0       # simulates yaw-sensor drift over time (slow sine)

    println("Publishing sensor information. Simulator running.")
    while ! is_shutdown()

        t = time()
        # update current state with a new row vector
        z_current[i,:]  = simModel(z_current[i-1,:]',u_current, dt, l_A,l_B)'
        z_real.t[i]     = t
        
        # Encoder measurements calculation
        dist_traveled += z_current[i,4]*dt #count the total traveled distance since the beginning of the simulation
        if dist_traveled - last_updated >= quarterCirc
            last_updated = dist_traveled
            FL += 1
            FR += 1
            BL += 0 #no encoder on back left wheel
            BR += 0 #no encoder on back right wheel
            enc_data = Encoder(FL, FR, BL, BR)
            publish(pub_enc, enc_data) #publish a message everytime the encoder counts up
        end

        # IMU measurements
        imu_data = Imu()
        imu_drift = sin(t/100*pi/2)     # drifts to 1 in 100 seconds
        yaw     = z_current[i,3] + 0*(randn()*0.05 + imu_drift)
        yaw_dot = (z_current[i,3]-z_current[i-1,3])/dt
        imu_data.orientation = geometry_msgs.msg.Quaternion(cos(yaw/2), sin(yaw/2), 0, 0)
        imu_data.angular_velocity = Vector3(0,0,yaw_dot)
        if i%2 == 0
            imu_meas.i += 1
            imu_meas.t[imu_meas.i] = t
            imu_meas.z[imu_meas.i,:] = [yaw yaw_dot]
            publish(pub_imu, imu_data)      # Imu format is defined by ROS, you can look it up by google "rosmsg Imu"
                                            # It's sufficient to only fill the orientation part of the Imu-type (with one quaternion)
        end

        # GPS measurements
        x = round(z_current[i,1]*100 + 0*randn()*2)       # Indoor gps measures in cm
        y = round(z_current[i,2]*100 + 0*randn()*2)
        if i % 7 == 0
            gps_meas.i += 1
            gps_meas.t[gps_meas.i] = t
            gps_meas.z[gps_meas.i,:] = [x y]
            gps_data = Vector3(x,y,0)
            publish(pub_gps, gps_data)
        end

        i += 1
        rossleep(loop_rate)
    end

    # Clean up buffers

    clean_up(gps_meas)
    clean_up(est_meas)
    clean_up(est_meas_dyn)
    clean_up(imu_meas)
    clean_up(cmd_log)
    z_real.z[1:i-1,:] = z_current[1:i-1,:]
    z_real.i = i
    clean_up(z_real)
    # Save simulation data to file
    log_path = "$(homedir())/simulations/output.jld"
    save(log_path,"gps_meas",gps_meas,"z",z_real,"estimate",est_meas,"estimate_dyn",est_meas_dyn,"imu_meas",imu_meas,"cmd_log",cmd_log)
    println("Exiting node... Saving data to $log_path. Simulated $((i-1)*dt) seconds.")
    #writedlm(log_path,z_current[1:i-1,:])
end

if ! isinteractive()
    main()
end
