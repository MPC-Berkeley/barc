 #!/usr/bin/env julia

 #=
	File name: low_level_controller.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed at UC
# Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu) and Greg Marcil (grmarcil@berkeley.edu). The cloud
# services integation with ROS was developed by Kiet Lam
# (kiet.lam@berkeley.edu). The web-server app Dator was based on an open source
# project by Bruce Wootton
# ---------------------------------------------------------------------------

using RobotOS


type LowLevelController
	motor_pwm::Float64
	servo_pwm::Float64
	ecu_pub::RobotOS.Publisher{barc.msg.ECU}
	ecu_cmd::ECU

	LowLevelController() = new()
end

function init!(low_level_controller::LowLevelController)
	low_level_controller.motor_pwm = 90
    low_level_controller.servo_pwm = 90
    low_level_controller.ecu_pub = Publisher("ecu_pwm", ECU, 
    										 queue_size = 1)::RobotOS.Publisher{barc.msg.ECU}
    low_level_controller.ecu_cmd = ECU()
end

function pwm_converter!(low_level_controller::LowLevelController, 
					    acceleration::Float64, steering_angle::Float64)
	# translate from SI units in vehicle model
    # to pwm angle units (i.e. to send command signal to actuators)
    # convert desired steering angle to degrees, saturate based on input limits

    node_name = get_name()[2 : end]

    if node_name == "agent_1"
        # println("AGENT 1 STEERING!!!!")
        low_level_controller.servo_pwm = 90.0 + 89.0 * Float64(steering_angle)
    else
        # println("AGENT 2 STEERING!!!!")
        low_level_controller.servo_pwm = 83.3 + 103.1 * Float64(steering_angle)
    end

    # compute motor command
    FxR = Float64(acceleration)

    if FxR == 0
        low_level_controller.motor_pwm = 90.0
    elseif FxR > 0
    	# using writeMicroseconds() in Arduino
        low_level_controller.motor_pwm = 91 + 6.5 * FxR   
        # Note: Barc doesn't move for u_pwm < 93
    else  # motor break / slow down
        low_level_controller.motor_pwm = 93.5 + 46.73 * FxR
    end
    
    update_arduino!(low_level_controller)
end

function neutralize!(low_level_controller::LowLevelController)
	println("Slowing down.")
    low_level_controller.motor_pwm = 40             # slow down first
    low_level_controller.servo_pwm = 90
    update_arduino!(low_level_controller)
    sleep(1)  # slow down for 1 sec
    println("Stopping.")
    low_level_controller.motor_pwm = 90
    update_arduino!(low_level_controller)
end

function update_arduino!(low_level_controller::LowLevelController)
    low_level_controller.ecu_cmd.header.stamp = get_rostime()
    low_level_controller.ecu_cmd.motor = low_level_controller.motor_pwm
    low_level_controller.ecu_cmd.servo = low_level_controller.servo_pwm
    publish(low_level_controller.ecu_pub, low_level_controller.ecu_cmd)
end
