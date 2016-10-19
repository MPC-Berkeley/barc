using JLD
using PyPlot

type Measurements{T}
    i::Int64          # measurement counter
    t::Array{Float64}       # time data
    z::Array{T}       # measurement values
end

const log_path_sim      = "$(homedir())/simulations/output.jld"                             # data from barc_simulation

# THIS FUNCTION EVALUATES DATA THAT WAS LOGGED BY THE SIMULATOR (INCLUDES "REAL" SIMULATION DATA)
# ***********************************************************************************************

function eval_sim()
    d_sim = load(log_path_sim)

    imu_meas    = d_sim["imu_meas"]
    gps_meas    = d_sim["gps_meas"]
    z           = d_sim["z"]
    cmd_log     = d_sim["cmd_log"]
    slip_a      = d_sim["slip_a"]

    t0 = imu_meas.t[1]

    figure()
    ax1=subplot(311)
    plot(z.t-t0,z.z,"-*")
    title("Real states")
    grid()
    legend(["x","y","v_x","v_y","psi","psi_dot","a","d_f"])
    subplot(312,sharex=ax1)
    plot(cmd_log.t-t0,cmd_log.z,"-*")
    title("Inputs")
    grid()
    legend(["u","d_f"])
    subplot(313,sharex=ax1)
    plot(slip_a.t-t0,slip_a.z,"-*")
    title("Slip angles")
    grid()
    legend(["a_f","a_r"])

    figure()
    plot(z.z[:,1],z.z[:,2],"-",gps_meas.z[:,1],gps_meas.z[:,2],".")
    grid(1)
    title("x-y-view")
    axis("equal")
    legend(["Real state","GPS meas"])
    
    figure()
    title("Comparison of psi")
    plot(imu_meas.t,imu_meas.z,"-x",z.t,z.z[:,5:6],)
    legend(["imu_psi","imu_psi_dot","real_psi","real_psi_dot"])
    grid()

    figure()
    title("Comparison of v")
    plot(z.t,z.z[:,3:4])
    legend(["real_xDot","real_yDot"])
    grid()

    figure()
    title("Comparison of x,y")
    plot(z.t,z.z[:,1:2],gps_meas.t,gps_meas.z)
    legend(["real_x","real_y","meas_x","meas_x"])
    grid()
end


# *****************************************************************
# ****** HELPER FUNCTIONS *****************************************
# *****************************************************************

function initPlot()         # prepares plots for export (to pdf, pgf, ...)
    linewidth = 0.4
    rc("axes", linewidth=linewidth)
    rc("lines", linewidth=linewidth, markersize=2)
    #rc("font", family="")
    rc("axes", titlesize="small", labelsize="small")        # can be set in Latex
    rc("xtick", labelsize="x-small")
    rc("xtick.major", width=linewidth/2)
    rc("ytick", labelsize="x-small")
    rc("ytick.major", width=linewidth/2)
    rc("legend", fontsize="small")
    rc("font",family="serif")
    rc("font",size=8)
    rc("figure",figsize=[4.5,3])
end
