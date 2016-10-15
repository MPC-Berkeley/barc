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
@rosimport barc.msg: ECU, pos_info
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
@rosimport sensor_msgs.msg: Imu
@rosimport marvelmind_nav.msg: hedge_pos
@rosimport std_msgs.msg: Float32
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using sensor_msgs.msg
using std_msgs.msg
using marvelmind_nav.msg
using JLD

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

function Quat2Euler(q::Array{Float64})
    sol = zeros(Float64,3)
    sol[1]   = atan2(2*(q[1]*q[2]+q[3]*q[4]),1-2*(q[2]^2+q[3]^2))
    sol[2]   = asin(2*(q[1]*q[3]-q[4]*q[2]))
    sol[3]   = atan2(2*(q[1]*q[4]+q[2]*q[3]),1-2*(q[3]^2+q[4]^2))
    return sol
end

function ECU_callback(msg::ECU,cmd_log::Measurements)
    cmd_log.i += 1
    cmd_log.t[cmd_log.i] = time()
    cmd_log.z[cmd_log.i,:] = convert(Array{Float64,1},[msg.motor;msg.servo])
    nothing
end

function IMU_callback(msg::Imu,imu_meas::Measurements)
    imu_meas.i += 1
    imu_meas.t[imu_meas.i]      = time()
    imu_meas.z[imu_meas.i,:]    = [msg.angular_velocity.x;msg.angular_velocity.y;msg.angular_velocity.z;
                                    Quat2Euler([msg.orientation.w;msg.orientation.x;msg.orientation.y;msg.orientation.z]);
                                    msg.linear_acceleration.x;msg.linear_acceleration.y;msg.linear_acceleration.z]::Array{Float64}
    nothing
end

function GPS_callback(msg::hedge_pos,gps_meas::Measurements)
    gps_meas.i += 1
    gps_meas.t[gps_meas.i]      = time()
    gps_meas.z[gps_meas.i,:]    = [msg.x_m;msg.y_m]
    nothing
end

function pos_info_callback(msg::pos_info,pos_info_log::Measurements)
    pos_info_log.i += 1
    pos_info_log.t[pos_info_log.i]      = time()
    pos_info_log.z[pos_info_log.i,:]    = [msg.s;msg.ey;msg.epsi;msg.v;msg.s_start;msg.x;msg.y;msg.v_x;msg.v_y;msg.psi;msg.psiDot]
    nothing
end

function vel_est_callback(msg::Float32Msg,vel_est_log::Measurements)
    vel_est_log.i += 1
    vel_est_log.t[vel_est_log.i]      = time()
    vel_est_log.z[vel_est_log.i]      = convert(Float64,msg.data)
    nothing
end

function main() 

    buffersize      = 60000
    gps_meas        = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))
    imu_meas        = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,9))
    cmd_log         = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,2))
    vel_est_log     = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize))
    pos_info_log    = Measurements{Float64}(0,zeros(buffersize),zeros(buffersize,11))

    gps_meas.t[1]       = time()
    imu_meas.t[1]       = time()
    cmd_log.t[1]        = time()
    pos_info_log.t[1]   = time()
    vel_est_log.t[1]    = time()
    # initiate node, set up publisher / subscriber topics
    init_node("barc_record")
    s1  = Subscriber("ecu", ECU, ECU_callback, (cmd_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.ECU}
    s2  = Subscriber("imu/data", Imu, IMU_callback, (imu_meas,), queue_size=1)::RobotOS.Subscriber{sensor_msgs.msg.Imu}
    s4  = Subscriber("hedge_pos", hedge_pos, GPS_callback, (gps_meas,), queue_size=1)::RobotOS.Subscriber{marvelmind_nav.msg.hedge_pos}
    s5  = Subscriber("pos_info", pos_info, pos_info_callback, (pos_info_log,), queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
    s6  = Subscriber("vel_est", Float32Msg, vel_est_callback, (vel_est_log,), queue_size=1)::RobotOS.Subscriber{std_msgs.msg.Float32Msg}

    dt = 0.1
    loop_rate = Rate(1/dt)

    println("Recorder running.")
    spin()                              # wait for callbacks until shutdown

    # Clean up buffers
    clean_up(gps_meas)
    clean_up(imu_meas)
    clean_up(cmd_log)
    clean_up(pos_info_log)
    clean_up(vel_est_log)

    # Save simulation data to file
    log_path = "$(homedir())/simulations/record-$(Dates.format(now(),"yyyy-mm-dd-HH-MM-SS")).jld"
    save(log_path,"gps_meas",gps_meas,"imu_meas",imu_meas,"cmd_log",cmd_log,"pos_info",pos_info_log,"vel_est",vel_est_log)
    println("Exiting node... Saving recorded data to $log_path.")
end

if ! isinteractive()
    main()
end
