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
@rosimport barc.msg: ECU, Z_DynBkMdl
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
@rosimport sensor_msgs.msg: Imu
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using sensor_msgs.msg
using JLD

#include("LMPC_lib/classes.jl")

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

buffersize      = 60000
gps_meas        = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))
imu_meas        = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,6))
est_meas_dyn    = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,6))
cmd_log         = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))

gps_meas.t[1]       = time()
imu_meas.t[1]       = time()
est_meas_dyn.t[1]   = time()
cmd_log.t[1]        = time()

function Quat2Euler(q::Array{Float64})
    sol = zeros(Float64,3)
    sol[1]   = atan2(2*(q[1]*q[2]+q[3]*q[4]),1-2*(q[2]^2+q[3]^2))
    sol[2]   = asin(2*(q[1]*q[3]-q[4]*q[2]))
    sol[3]   = atan2(2*(q[1]*q[4]+q[2]*q[3]),1-2*(q[3]^2+q[4]^2))
    return sol
end

function ECU_callback(msg::ECU)
    global cmd_log
    u_current = convert(Array{Float64,1},[msg.motor, msg.servo])
    cmd_log.i += 1
    cmd_log.t[cmd_log.i] = time()
    cmd_log.z[cmd_log.i,:] = u_current
    nothing
end

function est_dyn_callback(msg::Z_DynBkMdl)
    global est_meas_dyn
    est_meas_dyn.i += 1
    est_meas_dyn.t[est_meas_dyn.i]      = time()
    est_meas_dyn.z[est_meas_dyn.i,:]    = [msg.x, msg.y, msg.v_x, msg.v_y, msg.psi, msg.psi_dot]
    nothing
end

function IMU_callback(msg::Imu)
    global imu_meas
    imu_meas.i += 1
    imu_meas.t[imu_meas.i]      = time()
    imu_meas.z[imu_meas.i,:]    = [msg.angular_velocity.x;msg.angular_velocity.y;msg.angular_velocity.z;Quat2Euler([msg.orientation.w;msg.orientation.x;msg.orientation.y;msg.orientation.z])]::Array{Float64}
    nothing
end

function GPS_callback(msg::Vector3)
    global gps_meas
    gps_meas.i += 1
    gps_meas.t[gps_meas.i]      = time()
    gps_meas.z[gps_meas.i,:]    = [msg.x, msg.y, msg.z]
    nothing
end

function main() 
    # initiate node, set up publisher / subscriber topics
    init_node("barc_record")
    s1  = Subscriber("ecu", ECU, ECU_callback, queue_size=1)
    s2  = Subscriber("imu/data", Imu, IMU_callback, queue_size=1)
    s3  = Subscriber("state_estimate_dynamic", Z_DynBkMdl, est_dyn_callback, queue_size=1)
    s4  = Subscriber("indoor_gps", Vector3, GPS_callback, queue_size=1)

    dt = 0.1
    loop_rate = Rate(1/dt)

    println("Recorder running.")
    while ! is_shutdown()
        rossleep(loop_rate)
    end

    # Clean up buffers
    clean_up(gps_meas)
    clean_up(est_meas_dyn)
    clean_up(imu_meas)
    clean_up(cmd_log)

    # Save simulation data to file
    log_path = "$(homedir())/simulations/record.jld"
    save(log_path,"gps_meas",gps_meas,"estimate_dyn",est_meas_dyn,"imu_meas",imu_meas,"cmd_log",cmd_log)
    println("Exiting node... Saving recorded data to $log_path.")
end

if ! isinteractive()
    main()
end
