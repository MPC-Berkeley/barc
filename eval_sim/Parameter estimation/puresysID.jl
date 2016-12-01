
using PyPlot
using JLD

function main(code::AbstractString,n::Int64)
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    #log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    #d_rec       = load(log_path_record)
    d_lmpc      = load(log_path_LMPC)
    d_rec       = load(log_path_record)

    delay_df    = 5

    #n = 2

    #idx0 = (oldTraj.oldTraj[:,6,n].> 0) & (oldTraj.oldTraj[:,6,n].< 17.76)
    #oldTraj.oldTraj  = oldTraj.oldTraj[idx0,:,:]
    #oldTraj.oldInput = oldTraj.oldInput[idx0,:,:]
    oldTraj     = d_lmpc["oldTraj"]
    pos_info    = d_rec["pos_info"]
    imu_meas    = d_rec["imu_meas"]
    sz = size(oldTraj.oldTraj[oldTraj.oldTraj[:,1,n].<1000,:,n],1)
    oldTraj.oldTraj[1:sz,:,n] = smooth(oldTraj.oldTraj[1:sz,:,n],20)
    #oldTraj.oldInput[1:sz,:,n] = smooth(oldTraj.oldInput[1:sz,:,n],2)
    # plot(oldTraj.oldInput[1:sz,:,n])
    # for i=2:sz
    #     oldTraj.oldInput[i,:,n] = oldTraj.oldInput[i-1,:,n] + (oldTraj.oldInput[i,:,n]-oldTraj.oldInput[i-1,:,n])*0.5
    # end
    # plot(oldTraj.oldInput[1:sz,:,n],"--")
    #sz = 200
    idx_d = 20:5:sz-5
    idx   = 20:5:sz-10
    sz1 = size(idx,1)

    y_psi = zeros(5*sz1)
    A_psi = zeros(5*sz1,3)
    y_vy = zeros(5*sz1)
    A_vy = zeros(5*sz1,4)
    y_vx = zeros(5*sz1)
    A_vx = zeros(5*sz1,4)

    for i=0:4
        y_psi[(1:sz1)+i*sz1] = diff(oldTraj.oldTraj[idx_d+i,3,n])
        A_psi[(1:sz1)+i*sz1,:] = [oldTraj.oldTraj[idx+i,3,n]./oldTraj.oldTraj[idx+i,1,n] oldTraj.oldTraj[idx+i,2,n]./oldTraj.oldTraj[idx+i,1,n] oldTraj.oldInput[idx+i-delay_df,2,n]]
    end
    c_psi = (A_psi'*A_psi)\A_psi'*y_psi

    for i=0:4
        y_vy[(1:sz1)+i*sz1] = diff(oldTraj.oldTraj[idx_d+i,2,n])
        A_vy[(1:sz1)+i*sz1,:] = [oldTraj.oldTraj[idx+i,2,n]./oldTraj.oldTraj[idx+i,1,n] oldTraj.oldTraj[idx+i,1,n].*oldTraj.oldTraj[idx+i,3,n] oldTraj.oldTraj[idx+i,3,n]./oldTraj.oldTraj[idx+i,1,n] oldTraj.oldInput[idx+i-delay_df,2,n]]
    end
    c_vy = (A_vy'*A_vy)\A_vy'*y_vy

    for i=0:4
        y_vx[(1:sz1)+i*sz1] = diff(oldTraj.oldTraj[idx_d+i,1,n])
        A_vx[(1:sz1)+i*sz1,:] = [oldTraj.oldTraj[idx+i,2,n] oldTraj.oldTraj[idx+i,3,n] oldTraj.oldTraj[idx+i,1,n] oldTraj.oldInput[idx+i,1,n]]
    end
    c_vx = (A_vx'*A_vx)\A_vx'*y_vx

    println("psi:")
    println(c_psi)
    println("vy:")
    println(c_vy)
    println("vx:")
    println(c_vx)
    
    figure(1)
    plot(A_psi*c_psi)
    plot(y_psi,"--")
    #plot(A_psi,"--")
    title("Psi")

    figure(2)
    plot(A_vy*c_vy)
    plot(y_vy,"--")
    #plot(A_vy,"--")
    title("Vy")

    figure(3)
    plot(A_vx*c_vx)
    plot(y_vx,"--")
    #plot(A_vx,"--")
    title("Vx")

    # figure(4)
    # plot(oldTraj.oldTimes[:,1],oldTraj.oldTraj[:,3,1])
    # plot(pos_info.t,pos_info.z[:,11])
    # plot(imu_meas.t,imu_meas.z[:,3])
    nothing
end

function smooth(x,n)
    y = zeros(size(x))
    for i=1:size(x,1)
        start = max(1,i-n)
        fin = min(size(x,1),start + 2*n)
        y[i,:] = mean(x[start:fin,:],1)
    end
    return y
end