using PyPlot
using JLD

function test(path_acceleration::AbstractString,path_deceleration::AbstractString)

    
	files_acc = readdir(path_acceleration)
    files_dec = readdir(path_deceleration)

    a = zeros(length(files_acc) + length(files_dec))
    println("number of tests= ",length(a))

    ## Loop for acceleration 

    v_acceleration1 = []  #zeros(length(files))
    v_acceleration2 = []

    commands_acc = []  #zeros(length(files))

    for index = 1:length(files_acc)

        file = files_acc[index]

        log_path = "$(homedir())/open_loop/acceleration/$file"

        d_rec=load(log_path)

        cmd_pwm_log = d_rec["cmd_pwm_log"]
        vel_est     = d_rec["vel_est"]


        t0 = max(cmd_pwm_log.t[1],vel_est.t[1])
        t_end = min(cmd_pwm_log.t[end],vel_est.t[end])

        t = t0+0.1:.02:t_end-0.1
        v = zeros(length(t))
        cmd = zeros(length(t))

        for i=1:length(t)
            v[i] = vel_est.z[t[i].>vel_est.t,1][end]
            #v[i] = pos_info.z[t[i].>pos_info.t,15][end]
            cmd[i] = cmd_pwm_log.z[t[i].>cmd_pwm_log.t,1][end]
        end

        v_opt   = v[1:end]
        cmd_opt = cmd[1:end]
        v_opt   = v[cmd.>90]
        cmd_opt = cmd[cmd.>90]

        figure(1)

        plot(v_opt)
        grid("on")


        # println("length= ",length(files))
        # println("v_opt= ",length(v_opt))
        # println("cmd_opt= ",length(cmd_opt))
        # println("v_opt[end]= ",v_opt[end])
        # println("a[i]= ",a[i])
        # println("v_opt[end]/4= ",v_opt[end]/4)

        v_lms1 = zeros(length(v_opt)-1)
        v_lms2 = zeros(length(v_opt)-1)


        for j = 1:length(v_opt)-1

            v_lms1[j] = v_opt[j+1] - v_opt[j]
            v_lms2[j] = v_opt[j]

        end

        v_acceleration1 = vcat(v_acceleration1,v_lms1)
        v_acceleration2 = vcat(v_acceleration2,v_lms2)
        a[index] = v_opt[end]/4
        commands_acc = vcat(commands_acc,cmd_opt[1:end-1])


        # println("a= ",a)
        

        figure(2)

        plot(cmd_opt[1],a[index], "*")
        grid("on")

    end

    ## Loop for deceleration

    v_deceleration1 = []  #zeros(length(files))
    v_deceleration2 = []

    commands_dec = []  #zeros(length(files))

    for index = 1:length(files_dec)

        file = files_dec[index]

        log_path = "$(homedir())/open_loop/deceleration/$file"

        d_rec=load(log_path)

        cmd_pwm_log = d_rec["cmd_pwm_log"]
        vel_est     = d_rec["vel_est"]

        t0 = max(cmd_pwm_log.t[1],vel_est.t[1])
        t_end = min(cmd_pwm_log.t[end],vel_est.t[end])

        t = t0+0.1:.02:t_end-0.1
        v = zeros(length(t))
        cmd = zeros(length(t))

        for i=1:length(t)
            v[i] = vel_est.z[t[i].>vel_est.t,1][end]
            #v[i] = pos_info.z[t[i].>pos_info.t,15][end]
            cmd[i] = cmd_pwm_log.z[t[i].>cmd_pwm_log.t,1][end]
        end

        v_opt   = v[1:end]
        cmd_opt = cmd[1:end]
        v_opt   = v[cmd.<100]
        cmd_opt = cmd[cmd.<100]

        v_equal = 0
        flag = 0

        for i = 2:length(v_opt)

            if v_opt[i]==v_opt[i-1]
                v_equal = v_equal + 1
            else 
                v_equal = 0
            end

            if v_equal > 10
                flag = i
                break
            end
        end

        v_opt = v_opt[1:flag]
        cmd_opt = cmd_opt[1:flag]

        a[length(files_acc) + index] = (- v_opt[1])/4

        # plot(cmd_opt[5],a[length(files_acc) + index], "*")
        # grid("on")

        

        figure(3)

        plot(v_opt)
        grid("on")


        v_lms1 = zeros(length(v_opt)-1)
        v_lms2 = zeros(length(v_opt)-1)


        for j = 1:length(v_opt)-1

            v_lms1[j] = v_opt[j+1] - v_opt[j]
            v_lms2[j] = v_opt[j]

        end

        v_deceleration1 = vcat(v_deceleration1,v_lms1)
        v_deceleration2 = vcat(v_deceleration2,v_lms2)
        commands_dec = vcat(commands_dec,cmd_opt[1:end-1])




    end





    # Least Mean Square

    # println("length of v_acceleration1= ",length(v_acceleration1))
    # println("length of commands= ",length(commands))


    A = zeros(length(v_acceleration1)+length(v_deceleration1),5)
    B = zeros(length(v_acceleration1)+length(v_deceleration1),1)
    x = zeros(5,1)     #[c1_+;c2_+,cM]

    A[1:length(commands_acc),1] = commands_acc
    A[1:length(commands_acc),2] = 1
    A[1:length(v_acceleration2),3] = -v_acceleration2
    A[length(v_acceleration2)+1:end,3] = -v_deceleration2
    A[length(commands_acc)+1:end,4] = commands_dec
    A[length(commands_acc)+1:end,5] = 1

    B[1:length(v_acceleration1)] = v_acceleration1
    B[length(v_acceleration1)+1:end] = v_deceleration1 

    A = 0.02*A # 0.02 is the discretization


    x = inv(A'*A)*A'*B

    println("the computed constants are: ", x)

    # x_axis = (findmin(commands)[1]):0.02:(findmax(commands)[1])
    # y_axis = x[1]*x_axis + x[2] 
    # plot(x_axis,y_axis)

    


end
