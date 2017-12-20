#!/usr/bin/env julia

using RobotOS
@rosimport marvelmind_nav.msg: hedge_pos
rostypegen()
using barc.msg
#using data_service.msg
using geometry_msgs.msg
using JuMP
using Ipopt
using JLD

function gps_callback(msg::hedge_pos)

    position[:] = [msg.x_m; msg.y_m]

end

function main()

