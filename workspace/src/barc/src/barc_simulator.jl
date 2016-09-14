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
@rosimport barc.msg: ECU, pos_info, Encoder, Ultrasound, Z_KinBkMdl, Logging
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
@rosimport sensor_msgs.msg: Imu
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using sensor_msgs.msg

u_current = zeros(2,1)

function simModel(z,u,dt,l_A,l_B)

   # kinematic bicycle model
   # u[1] = acceleration
   # u[2] = steering angle

    bta = atan(l_A/(l_A+l_B)*tan(u[2]))

    zNext = z
    zNext[1] = z[1] + dt*(z[4]*cos(z[3]+bta))       # x
    zNext[2] = z[2] + dt*(z[4]*sin(z[3] + bta))      # y
    zNext[3] = z[3] + dt*(z[4]/l_B*sin(bta))        # psi
    zNext[4] = z[4] + dt*(u[1] - 0.63 * z[4]^2 * sign(z[4]))                     # v

    return zNext
end


function ECU_callback(msg::ECU)

    global u_current
    u_current = [msg.motor, msg.servo] 
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

    z_current = zeros(60000,4)

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


    println("Publishing sensor information. Simulator running.")
    while ! is_shutdown()

        # update current state with a new row vector
        z_current[i,:] = simModel(z_current[i-1,:]',u_current, dt, l_A,l_B)'
        dist_traveled += z_current[i,4]*dt


        
        # Encoder measurements calculation
        dist_traveled += z_current[i,4]*dt #count the total traveled distance since the beginning of the simulation
        if dist_traveled - last_updated >= quarterCirc
            last_updated = dist_traveled
            FL += 1
            FR += 1
            BL += 1
            BR += 0 #no encoder on back right wheel
            enc_data = Encoder(FL, FR, BL, BR)
            publish(pub_enc, enc_data) #publish a message everytime the encoder counts up
        end

        # IMU measurements
        imu_data = Imu()
        imu_data.orientation = geometry_msgs.msg.Quaternion(cos(z_current[i,3]/2)+randn()*0.01, sin(z_current[i,3]/2)+randn()*0.01, 0, 0)
        if i%2 == 0
            publish(pub_imu, imu_data)      # Imu format is defined by ROS, you can look it up by google "rosmsg Imu"
                                            # It's sufficient to only fill the orientation part of the Imu-type (with one quaternion)
        end

        # GPS measurements
        x = z_current[i,1]*100 + randn()       # Indoor gps measures in cm
        y = z_current[i,2]*100 + randn()
        if i % 7 == 0
            gps_data = Vector3(x,y,0)
            publish(pub_gps, gps_data)
        end

        i += 1
        rossleep(loop_rate)
    end

    # Save simulation data to file
    log_path = "$(homedir())/simulations/output.txt"
    println("Exiting node... Saving data to $log_path. Simulated $((i-1)*dt) seconds.")
    writedlm(log_path,z_current[1:i-1,:])
end

if ! isinteractive()
    main()
end
