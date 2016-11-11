using PyPlot
using JLD

function main(code::AbstractString)
    log_path_record = "$(homedir())/open_loop/output-record-$(code).jld"
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    d_rec = load(log_path_record)

    imu_meas    = d_rec["imu_meas"]
    t0 = imu_meas.t[1]
    sz = size(imu_meas.t,1)
    a_norm = zeros(sz,3)
    for i=1:sz
        a_norm[i,:] = (rotMatrix('y',-imu_meas.z[i,5])*rotMatrix('x',-imu_meas.z[i,4])*imu_meas.z[i,7:9]')'
    end
    figure()
    plot(imu_meas.t-t0,imu_meas.z[:,7:8])
    grid("on")
    figure()
    plot(imu_meas.t-t0,a_norm[:,1:2])
    grid("on")
    println("Mean raw: $(mean(imu_meas.z[:,8])), std raw: $(std(imu_meas.z[:,8]))")
    println("Mean filtered: $(mean(a_norm[:,2])), std filtered: $(std(a_norm[:,2]))")
end

function rotMatrix(s::Char,deg::Float64)
    A = zeros(3,3)
    if s=='x'
        A = [1 0 0;
             0 cos(deg) sin(deg);
             0 -sin(deg) cos(deg)]
    elseif s=='y'
        A = [cos(deg) 0 -sin(deg);
             0 1 0;
             sin(deg) 0 cos(deg)]
    elseif s=='z'
        A = [cos(deg) sin(deg) 0
             -sin(deg) cos(deg) 0
             0 0 1]
    else
        warn("Wrong angle for rotation matrix")
    end
    return A
end
