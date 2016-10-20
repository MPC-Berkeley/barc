#!/usr/bin/env julia

using RobotOS
@rosimport barc.msg: ECU
rostypegen()
using barc.msg


# This type contains measurement data (time, values and a counter)
type Measurements{T}
    i::Int64                # measurement counter
    t::Array{Float64}       # time data (when it was received by this recorder)
    t_msg::Array{Float64}   # time that the message was sent
    z::Array{T}             # measurement values
end

# This function cleans the zeros from the type above once the simulation is finished
function clean_up(m::Measurements)
    m.t     = m.t[1:m.i-1]
    m.t_msg = m.t_msg[1:m.i-1]
    m.z     = m.z[1:m.i-1,:]
end


function main()
    # Initialize ROS node and topics
    init_node("mpc_traj")
    loop_rate = Rate(10)
    pub                         = Publisher("ecu", ECU, queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
    t0_ros = get_rostime()
    t0 = to_sec(t0_ros)
    cmd                         = ECU()      # command type
    cmd.motor = 0
    cmd.servo = 0
    cmd.header.stamp = get_rostime()
    println("Starting open loop control.")
    # Start node
    while ! is_shutdown()
        t = to_sec(get_rostime())-t0
        if t <= 3
            cmd.motor = 0
            cmd.servo = 0
        elseif t <= 5
            cmd.motor = 1
            cmd.servo = 0
        else
            cmd.motor = 0
            cmd.servo = 0
        end

        cmd.header.stamp = get_rostime()
        publish(pub, cmd)  

        rossleep(loop_rate)
    end
    println("Exiting open loop control.")
end

if ! isinteractive()
    main()
end
