using PyPlot
using JLD

function test(path::AbstractString)

    
	files = readdir(path)

    a = zeros(length(files))

    v_final = zeros(length(files))

    commands = zeros(length(files))

    for index = 1:length(files)

        file = files[index]

        log_path = "$(homedir())/open_loop/$file"

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

        # println("length= ",length(files))
        # println("v_opt= ",v_opt)
        # println("v_opt[end]= ",v_opt[end])
        # println("a[i]= ",a[i])
        # println("v_opt[end]/4= ",v_opt[end]/4)

        v_final[index] = v_opt[end]
        a[index] = v_opt[end]/4
        commands[index]=cmd_opt[end]


        println("a= ",a)
        println("number of tests= ",length(a))

        figure(2)

        plot(cmd_opt[1],a[index], "*")

    end

    # Least Mean Square

    A = ones(length(files),3)
    B = zeros(length(files),1)
    x = zeros(3,1)     #[c1_+;c2_+,cM]

    A[:,1] = commands
    A[:,3] = -v_final

    B = a 

    x = inv(A'*A)*A'*B

    println("****quick check of the results****")

    a_test = x[1]*commands[1] + x[2] - x[3]*v_final[1]

    println("a_test= ",a_test)


end
