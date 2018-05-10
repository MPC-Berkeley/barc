# simple debugging
using JLD
using PyPlot
using PyCall
using QuickHull

@pyimport matplotlib.patches as patch
@pyimport matplotlib.transforms as transforms
@pyimport matplotlib.animation as animation
@pyimport matplotlib as mpl
@pyimport matplotlib.collections as collections 

include("barc_lib/classes.jl")
include("barc_lib/LMPC/functions.jl")

run_time = Dates.format(now(),"yyyy-mm-dd")
data        = load("$(homedir())/simulations/LMPC-$(run_time)-$(ARGS[2]).jld")
oldSS       = data["oldSS"]
selectedStates = data["selectedStates"]
solHistory  = data["solHistory"]
track       = data["track"]
modelParams = data["modelParams"]
mpcParams   = data["mpcParams"]
log_cvx     = data["log_cvx"]   
log_cvy     = data["log_cvy"]
log_cpsi    = data["log_cpsi"]
lapStatus   = LapStatus(1,1,false,false,0.3)

# STATE AND SYS_ID PARAMETERS PLOT FOR LAPS DONE
if ARGS[1] == "record" || ARGS[1]=="both"
    figure("Record") # PLOT OF STATE
    i = 1; plot_way = [3,2]
    while solHistory.cost[i] > 10
        i == 1 ? current_x = 0 : current_x = sum(solHistory.cost[1:i-1])
        x_len = Int(solHistory.cost[i])
        subplot(3,2,1)
        plot(current_x+1:current_x+x_len,solHistory.z[1:x_len,i,1,4],color="blue")
        plot([current_x,current_x],[0,2.5],color="grey",linestyle="--")
        ylabel("vx")

        subplot(3,2,3)
        plot(current_x+1:current_x+x_len,solHistory.z[1:x_len,i,1,5],color="blue")
        plot([current_x,current_x],[-0.3,0.3],color="grey",linestyle="--")
        ylabel("vy")

        subplot(3,2,5)
        plot(current_x+1:current_x+x_len,solHistory.z[1:x_len,i,1,5],color="blue")
        plot([current_x,current_x],[-3,3],color="grey",linestyle="--")
        ylabel("psi_dot")
        i += 1
    end
    # PLOT OF SYS_ID PARAMETERS
    # figure(figsize=(15,10))
    i = 1
    while solHistory.cost[i] > 10
        i == 1 ? current_x = 0 : current_x = sum(solHistory.cost[1:i-1])
        x_len = Int(solHistory.cost[i])
        subplot(3,2,2)
        for j=1:3
            if i == 1
                if j == 1
                    plot(current_x+1:current_x+x_len,log_cvx[1:x_len,1,j,i],color="blue",label="cvx_$j")
                elseif j == 2
                    plot(current_x+1:current_x+x_len,log_cvx[1:x_len,1,j,i],color="red",label="cvx_$j")
                elseif j == 3
                    plot(current_x+1:current_x+x_len,log_cvx[1:x_len,1,j,i],color="green",label="cvx_$j")
                end
            else
                if j == 1
                    plot(current_x+1:current_x+x_len,log_cvx[1:x_len,1,j,i],color="blue")
                elseif j == 2
                    plot(current_x+1:current_x+x_len,log_cvx[1:x_len,1,j,i],color="red")
                elseif j == 3
                    plot(current_x+1:current_x+x_len,log_cvx[1:x_len,1,j,i],color="green")
                end
            end
            plot([current_x,current_x],[-1,1],color="grey",linestyle="--")
        end
        legend(); ylabel("cvx")

        subplot(3,2,4)
        for j=1:4
            if i == 1
                if j == 1
                    plot(current_x+1:current_x+x_len,log_cvy[1:x_len,1,j,i],color="blue",label="cvy_$j")
                elseif j == 2
                    plot(current_x+1:current_x+x_len,log_cvy[1:x_len,1,j,i],color="red",label="cvy_$j")
                elseif j == 3
                    plot(current_x+1:current_x+x_len,log_cvy[1:x_len,1,j,i],color="green",label="cvy_$j")
                elseif j == 4
                    plot(current_x+1:current_x+x_len,log_cvy[1:x_len,1,j,i],color="orange",label="cvy_$j")
                end
            else
                if j == 1
                    plot(current_x+1:current_x+x_len,log_cvy[1:x_len,1,j,i],color="blue")
                elseif j == 2
                    plot(current_x+1:current_x+x_len,log_cvy[1:x_len,1,j,i],color="red")
                elseif j == 3
                    plot(current_x+1:current_x+x_len,log_cvy[1:x_len,1,j,i],color="green")
                elseif j == 4
                    plot(current_x+1:current_x+x_len,log_cvy[1:x_len,1,j,i],color="orange")
                end
            end
            plot([current_x,current_x],[-1,1],color="grey",linestyle="--")
        end
        legend(); ylabel("cvy")

        subplot(3,2,6)
        for j=1:3
            if i == 1
                if j == 1
                    plot(current_x+1:current_x+x_len,log_cpsi[1:x_len,1,j,i],color="blue",label="cpsi_$j")
                elseif j == 2
                    plot(current_x+1:current_x+x_len,log_cpsi[1:x_len,1,j,i],color="red",label="cpsi_$j")
                elseif j == 3
                    plot(current_x+1:current_x+x_len,log_cpsi[1:x_len,1,j,i],color="green",label="cpsi_$j")
                end
            else
                if j == 1
                    plot(current_x+1:current_x+x_len,log_cpsi[1:x_len,1,j,i],color="blue")
                elseif j == 2
                    plot(current_x+1:current_x+x_len,log_cpsi[1:x_len,1,j,i],color="red")
                elseif j == 3
                    plot(current_x+1:current_x+x_len,log_cpsi[1:x_len,1,j,i],color="green")
                end
            end
            plot([current_x,current_x],[-4,4],color="grey",linestyle="--")
        end
        legend(); ylabel("cpsi")
        i += 1
    end
    show()
end

# TRAJECTORY PLOT FOR ALL THE LAPS DONE
if ARGS[1] == "trajectory" || ARGS[1]=="both"
    i = 1
    fig = figure("Trajectory")
    axs = fig[:add_subplot](1, 1, 1); line = nothing
    plot(track.xy[:, 1],       track.xy[:, 2],       color="grey",alpha=0.4)
    plot(track.bound1xy[:, 1], track.bound1xy[:, 2], color="red")
    plot(track.bound2xy[:, 1], track.bound2xy[:, 2], color="red")
    while solHistory.cost[i] > 10
        x_len = Int(solHistory.cost[i])
        v = sqrt(solHistory.z[1:x_len,i,1,4].^2 + solHistory.z[1:x_len,i,1,5].^2)
        state = reshape(solHistory.z[1:x_len,i,1,:],x_len,6)
        (x,y) = trackFrame_to_xyFrame(state,track)

        points = reshape([x y], (size(x)[1], 1, 2))
        points_1 = points[1 : end - 1, :, :]
        points_2 = points[2 : end, :, :]
        segments = cat(2, points_1, points_2)

        (v_min,~) = findmin(v)
        (v_max,~) = findmax(v)
        # max should max(V_MAX), min = 0
        norm = plt[:Normalize](v_min, v_max)

        lc = collections.LineCollection(segments, cmap="jet", norm=norm)
        # Set the values used for colormapping
        lc[:set_array](v)
        lc[:set_linewidth](2)
        line = axs[:add_collection](lc)
        i+=1
    end
    axis("equal")
    # grid("on")
    fig[:colorbar](line, ax=axs)
    xlabel("x [m]")
    ylabel("y [m]")
    show()
end

function set_limits(ax, solHistory::SolHistory,index::Int64)
    min_val = findmin(solHistory.z[:, :, :, index])[1]
    max_val = findmax(solHistory.z[:, :, :, index])[1]
    ax[:set_ylim]([min_val - 0.1, max_val + 0.1])
end

function update_limits(ax, x_pred, x_true, x_select)
    ax[:set_xlim]([min(findmin(x_pred)[1],findmin(x_true)[1],findmin(x_select)[1]) - 0.1,
                   max(findmax(x_pred)[1],findmax(x_true)[1],findmax(x_select)[1]) + 0.1])
end

function plot_prediction(ax, x, y, c)
    prediction, = ax[:plot](x, y, color=c, marker="*")
    return prediction
end

function update_prediction(plt_h, x, y)
    plt_h[:set_data](x, y)
end

function plot_selection(ax, x, y, c)
    # Plot convex hull of the selected_states
    conv_hull_temp = qhull(x, y)
    conv_hull = zeros(size(conv_hull_temp[1])[1] + 1, 2)
    conv_hull[1 : end - 1, 1] = conv_hull_temp[1]
    conv_hull[1 : end - 1, 2] = conv_hull_temp[2]
    conv_hull[end, 1] = conv_hull[1, 1]
    conv_hull[end, 2] = conv_hull[1, 2]

    convex_hull = patch.Polygon(conv_hull, fill=true, color=c, alpha=0.2)
    ax[:add_artist](convex_hull)

    plt_hull_bounds, = ax[:plot](conv_hull[:, 1], conv_hull[:, 2], color=c, linestyle="-", lw=2.0)
    plt_states, = ax[:plot](x, y, color=c, marker="o")

    return convex_hull, plt_hull_bounds, plt_states
end

function update_selection(convex_hull, plt_hull_bounds, plt_states, x, y)
    # Plot convex hull of the selected_states
    conv_hull_temp = qhull(x, y)
    conv_hull = zeros(size(conv_hull_temp[1])[1] + 1, 2)
    conv_hull[1 : end - 1, 1] = conv_hull_temp[1]
    conv_hull[1 : end - 1, 2] = conv_hull_temp[2]
    conv_hull[end, 1] = conv_hull[1, 1]
    conv_hull[end, 2] = conv_hull[1, 2]

    convex_hull[:set_xy](conv_hull)
    plt_hull_bounds[:set_data](conv_hull[:, 1], conv_hull[:, 2])
    plt_states[:set_data](x, y)
end

function plot_closed_loop(ax, x::Array{Float64,1}, y::Array{Float64,1}, c)
    closed_loop, = ax[:plot](x, y, color=c, linestyle="--", lw=2)
    return closed_loop
end

function update_closed_loop(closed_loop_plt, x::Array{Float64,1}, y::Array{Float64,1}, bufferSize::Int64)
    x_full = vcat(x,NaN*ones(bufferSize-size(x,1)))
    y_full = vcat(y,NaN*ones(bufferSize-size(y,1)))
    closed_loop_plt[:set_data](x_full, y_full)
end

if ARGS[1] == "prediction"
    plt[:ion]()

    # Create figure and subplots
    fig = figure("Prediction ")
    ax_s_ey = fig[:add_subplot](2, 3, 1)
    xlabel("s [m]")
    ylabel("e_y [m]")

    ax_s_epsi = fig[:add_subplot](2, 3, 2)
    xlabel("s [m]")
    ylabel("e_psi [rad]")

    ax_s_psidot = fig[:add_subplot](2, 3, 3)
    xlabel("s [m]")
    ylabel("psi_dot [rad / s]")

    ax_s_vx = fig[:add_subplot](2, 3, 4)
    xlabel("s [m]")
    ylabel("v_x [m / s]")

    ax_s_vy = fig[:add_subplot](2, 3, 5)
    xlabel("s [m]")
    ylabel("v_y [m / s]")

    ax = [ax_s_ey, ax_s_epsi, ax_s_vx, ax_s_vy, ax_s_psidot]

    # SET THE Y-LIMITS FOR EACH SUBPLOT
    for i = 1 : 5
        set_limits(ax[i], solHistory, i+1)
    end
    
    # INITIALIZE THE PLOTING FOR MPC PREDICTION
    pre_hs = []
    for i = 1 : 5
        pre_h = plot_prediction(ax[i], NaN*ones(size(solHistory.z,3)), NaN*ones(size(solHistory.z,3)), "blue")
        pre_hs = [pre_hs; pre_h]
    end
    
    # INITIALIZE THE PLOTTING FOR TRUE PREDICTION WHICH NEEDS TO BE TURNED OFF DURING EXPERIMENT
    true_hs = []
    for i = 1 : 5
        true_h = plot_prediction(ax[i], NaN*ones(size(solHistory.z,3)), NaN*ones(size(solHistory.z,3)), "black")
        true_hs = [true_hs; true_h]
    end
    
    # INITIALIZE THE PLOTTING FOR SAFESET
    conv_hulls = []
    hull_bounds = []
    plt_states = []
    for i = 1 : 5
        conv_hull, hull_bound, plt_state = plot_selection(ax[i], selectedStates.selStates[:, 1], 
                                                                 selectedStates.selStates[:, i + 1], "blue")
        conv_hulls = [conv_hulls; conv_hull]
        hull_bounds = [hull_bounds; hull_bound]
        plt_states = [plt_states; plt_state]
    end

    # INITIALIZE THE PLOT FOR THE CLOSE LOOP TRAJECTORY
    closed_loop_plts = []; bufferSize = 500
    for i = 1 : 5
        closed_loop_plt = plot_closed_loop(ax[i], NaN*ones(bufferSize), NaN*ones(bufferSize), "grey")
        closed_loop_plts = [closed_loop_plts; closed_loop_plt]
    end

    # UPDATE AND SAVE THE PLOT FOR EVERY LMPC LAPS
    lap = 2 + max(selectedStates.Nl,selectedStates.feature_Nl)  # starting lap of LMPC
    while solHistory.cost[lap] > 10
        lapStatus.currentLap = lap

        # UPDATE THE PLOT OF THE CURRENT LAP CLOSE LOOP TRAJECTORY
        for i = 1 : 5
            update_closed_loop(closed_loop_plts[i], solHistory.z[1:Int(solHistory.cost[lap]),lap,1,1],
                                                    solHistory.z[1:Int(solHistory.cost[lap]),lap,1,i+1], bufferSize)
        end

        for j = 1:Int(solHistory.cost[lap])
            # SELECT THE SAVE SET
            selectedStates = find_SS(oldSS,selectedStates,reshape(solHistory.z[j,lap,:,:],size(solHistory.z,3),size(solHistory.z,4)),
                                     lapStatus,modelParams,mpcParams,track)
            
            # SIMULATE THE TRUE MODEL, WHICH SHOULD BE TURNED OFF FOR EXPERIMENTS
            true_state, ~, ~ = car_pre_dyn(reshape(solHistory.z[j,lap,1,:],1,size(solHistory.z,4)),
                                           reshape(solHistory.u[j,lap,:,:],size(solHistory.u,3),size(solHistory.u,4)),track,modelParams,6)

            # UPDATE THE PLOTTING DATA
            for i = 1 : 5
                update_selection(conv_hulls[i], hull_bounds[i], plt_states[i], 
                                 selectedStates.selStates[:,1],selectedStates.selStates[:,1+i])
                update_prediction(pre_hs[i], solHistory.z[j,lap,:,1], solHistory.z[j,lap,:,1+i])
                update_prediction(true_hs[i], true_state[:,1], true_state[:,i+1])
                update_limits(ax[i],solHistory.z[j,lap,:,1],true_state[:,1],selectedStates.selStates[:,1])
            end
            fig[:savefig]("$(homedir())/simulations/animations/lap$(lap)_iteration$(j).png", dpi=100)
            println("lap$(lap)_iteration$(j)")
        end
        lap += 1
    end
end
