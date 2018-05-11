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

const select_flag = false

run_time = Dates.format(now(),"yyyy-mm-dd")
data        = load("$(homedir())/simulations/LMPC-$(run_time)-$(ARGS[2]).jld")
oldSS       = data["oldSS"]
selectedStates = data["selectedStates"]
solHistory  = data["solHistory"]
selectHistory = data["selectHistory"]
selectFeatureHistory = data["selectFeatureHistory"]
statusHistory = data["statusHistory"]
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
        plot(current_x+1:current_x+x_len,solHistory.z[1:x_len,i,1,6],color="blue")
        plot([current_x,current_x],[-3,3],color="grey",linestyle="--")
        ylabel("psi_dot")
        i += 1
        if i>length(solHistory.cost)
            break
        end
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
        if i>length(solHistory.cost)
            break
        end
    end
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
        if i>length(solHistory.cost)
            break
        end
    end
    axis("equal")
    # grid("on")
    fig[:colorbar](line, ax=axs)
    xlabel("x [m]")
    ylabel("y [m]")
end
show()

function set_limits(ax, solHistory::SolHistory,index::Int64)
    i = 1; min_val = 0; max_val = 0
    while solHistory.cost[i]>10
        min_val = min(findmin(solHistory.z[:, i, :, index])[1],min_val)
        max_val = max(findmax(solHistory.z[:, i, :, index])[1],max_val)
        i+=1
        if i>length(solHistory.cost)
            break
        end
    end
    ax[:set_ylim]([min_val - 0.1, max_val + 0.1])
end

function update_limits(ax, x_pred::Array{Float64}, x_true::Array{Float64,1}, x_select::Array{Float64,1})
    ax[:set_xlim]([min(findmin(x_pred)[1],findmin(x_true)[1],findmin(x_select)[1]) - 0.1,
                   max(findmax(x_pred)[1],findmax(x_true)[1],findmax(x_select)[1]) + 0.1])
    # println("pre",findmax(x_pred)[1])
    # println(findmax(x_true)[1])
    # println(findmax(x_select)[1])
end

function plot_prediction(ax, x::Array{Float64,1}, y::Array{Float64,1}, c)
    prediction, = ax[:plot](x, y, color=c, marker="*")
    return prediction
end

function update_prediction(plt_h, x::Array{Float64}, y::Array{Float64}, track::Track)
    plt_h[:set_data](x, y)
end

function plot_selection(ax, x::Array{Float64,1}, y::Array{Float64,1}, c)
    # Plot convex hull of the selected_states
    conv_hull_temp = qhull(x, y)
    conv_hull = zeros(size(conv_hull_temp[1])[1] + 1, 2)
    conv_hull[1 : end - 1, 1] = conv_hull_temp[1]
    conv_hull[1 : end - 1, 2] = conv_hull_temp[2]
    conv_hull[end, 1] = conv_hull[1, 1]
    conv_hull[end, 2] = conv_hull[1, 2]

    convex_hull = patch.Polygon(conv_hull, fill=true, color=c, alpha=0.2)
    ax[:add_artist](convex_hull)

    plt_hull_bounds, = ax[:plot](conv_hull[:, 1], conv_hull[:, 2], color=c, linestyle="-", lw=2.0, alpha=0.3)
    plt_states, = ax[:plot](x, y, color=c, marker="o", alpha=0.3)

    return convex_hull, plt_hull_bounds, plt_states
end

function update_selection(convex_hull, plt_hull_bounds, plt_states, x::Array{Float64,1}, y::Array{Float64,1})
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
    fig = figure("Prediction",figsize=(20,10))
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

    ax_s_c = fig[:add_subplot](2, 3, 6)
    xlabel("s [m]")
    ylabel("curvature [1 / m]")

    ax = [ax_s_ey, ax_s_epsi, ax_s_vx, ax_s_vy, ax_s_psidot]

    # PLOT FINISHING LINE
    for axe in ax
        axe[:plot]([0,0],[-10,10],color="black", linestyle="-")
        axe[:plot]([track.s,track.s],[-10,10],color="black", linestyle="-")
    end
    # PLOT THE BOUNDARY FOR STRAIGHT LINE AND CURVE
    for axe in ax
        for i = 1:size(track.curvature,1)-1
            if (track.curvature[i] == 0 && track.curvature[i+1] != 0)
                axe[:plot]([i-1,i-1]*track.ds,[-10,10],color="grey", linestyle="-")
            elseif (track.curvature[i] != 0 && track.curvature[i+1] == 0)
                axe[:plot]([i-1,i-1]*track.ds,[-10,10],color="grey", linestyle="-.")
            end
        end
    end
    # PLOT THE TRACK BOUNDARY ON e_y
    x = (track.idx-1)*track.ds
    y = track.w/2*ones(size(x,1))
    ax_s_ey[:plot](x, -y, color="grey", linestyle="--")
    ax_s_ey[:plot](x,0*y, color="grey", linestyle="--")
    ax_s_ey[:plot](x,  y, color="grey", linestyle="--")

    # SET THE Y-LIMITS FOR EACH SUBPLOT
    for i = 1 : 5
        set_limits(ax[i], solHistory, i+1)
    end
    ax_s_vy[:set_ylim]([-0.15, 0.15])
    ax_s_c[:set_ylim]([-1.2,1.2])
    
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
        conv_hull, hull_bound, plt_state = plot_selection(ax[i], NaN*ones(size(selectedStates.selStates[:, 1])), 
                                                                 NaN*ones(size(selectedStates.selStates[:, 1])), "blue")
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

    # INITIALIZE THE PLOT FOR CURVATURE
    curvature_plt, = ax_s_c[:plot](NaN*ones(mpcParams.N+1), NaN*ones(mpcParams.N+1), color="blue", marker="o")

    fig_select = figure("Selected points",figsize=(20,10))
    ax_track_select = fig_select[:add_subplot](1, 1, 1)
    ax_track_select[:plot](track.bound1xy[:,1],track.bound1xy[:,2],color="black")
    ax_track_select[:plot](track.bound2xy[:,1],track.bound2xy[:,2],color="black")
    select_plt, = ax_track_select[:plot](NaN*ones(size(selectedStates.selStates[:, 1])),
                                         NaN*ones(size(selectedStates.selStates[:, 1])),"go",alpha=0.3)
    car_plt, = ax_track_select[:plot](NaN,NaN,"ko",markersize=5,alpha=0.3)

    xlabel("x [m]")
    ylabel("y [m]")
    axis("equal")

    # UPDATE AND SAVE THE PLOT FOR EVERY LMPC LAPS
    if length(ARGS) == 2
        lap = 1
    elseif ARGS[3] == "LMPC"
        lap = 2 + max(selectedStates.Nl,selectedStates.feature_Nl)  # starting lap of LMPC
    end

    while solHistory.cost[lap] > 10
        lapStatus.currentLap = lap

        # UPDATE THE PLOT OF THE CURRENT LAP CLOSE LOOP TRAJECTORY
        for i = 1 : 5
            update_closed_loop(closed_loop_plts[i], solHistory.z[1:Int(solHistory.cost[lap]),lap,1,1],
                                                    solHistory.z[1:Int(solHistory.cost[lap]),lap,1,i+1], bufferSize)
        end

        if lap <= 1 + max(selectedStates.Nl,selectedStates.feature_Nl)
            # UPDATE THE PLOT OF THE CURRENT LAP CLOSE LOOP TRAJECTORY
            for j = 1:Int(solHistory.cost[lap])
                # SIMULATE THE TRUE MODEL, WHICH SHOULD BE TURNED OFF FOR EXPERIMENTS
                true_state, ~, ~ = car_pre_dyn(reshape(solHistory.z[j,lap,1,:],1,size(solHistory.z,4)),
                                               reshape(solHistory.u[j,lap,:,:],size(solHistory.u,3),size(solHistory.u,4)),track,modelParams,6)

                # UPDATE THE PLOTTING DATA
                for i = 1 : 5
                    update_prediction(pre_hs[i], solHistory.z[j,lap,:,1], solHistory.z[j,lap,:,1+i], track)
                    update_prediction(true_hs[i], true_state[:,1], true_state[:,i+1], track)
                    update_limits(ax[i],solHistory.z[j,lap,:,1],true_state[:,1],NaN*ones(mpcParams.N+1))
                end
                
                # UPDATE THE CURVATURE PLOT
                curvature=curvature_prediction(reshape(solHistory.z[j,lap,:,:],size(solHistory.z[j,lap,:,:],3),size(solHistory.z[j,lap,:,:],4)),track)
                update_prediction(curvature_plt,solHistory.z[j,lap,:,1],curvature,track)
                ax_s_c[:set_xlim]([findmin(solHistory.z[j,lap,:,1])[1]-0.1, findmax(solHistory.z[j,lap,:,1])[1] + 0.1])

                fig[:savefig]("$(homedir())/simulations/animations/lap$(lap)_iteration$(j).png", dpi=100)
                println("lap$(lap)_iteration$(j)")
            end
        else
            for j = 1:Int(solHistory.cost[lap])
                # SIMULATE THE TRUE MODEL, WHICH SHOULD BE TURNED OFF FOR EXPERIMENTS
                true_state, ~, ~ = car_pre_dyn(reshape(solHistory.z[j,lap,1,:],1,size(solHistory.z,4)),
                                               reshape(solHistory.u[j,lap,:,:],size(solHistory.u,3),size(solHistory.u,4)),track,modelParams,6)

                # UPDATE THE PLOTTING DATA
                for i = 1 : 5
                    update_selection(conv_hulls[i], hull_bounds[i], plt_states[i], 
                                     reshape(selectHistory[j,lap,:,1],size(selectHistory[j,lap,:,1],3)),
                                     reshape(selectHistory[j,lap,:,1+i],size(selectHistory[j,lap,:,1],3)))
                    update_prediction(pre_hs[i], solHistory.z[j,lap,:,1], solHistory.z[j,lap,:,1+i], track)
                    update_prediction(true_hs[i], true_state[:,1], true_state[:,i+1], track)
                    update_limits(ax[i],solHistory.z[j,lap,:,1],true_state[:,1],reshape(selectHistory[j,lap,:,1],size(selectHistory,3)))
                    
                    if select_flag
                        # FEATURE DATA PLOTTING UPDATE
                        (z_iden_x, z_iden_y) = trackFrame_to_xyFrame(reshape(selectFeatureHistory[j,lap,:,:],size(selectFeatureHistory,3),size(selectFeatureHistory,4)),track)
                        update_prediction(select_plt,z_iden_x,z_iden_y,track)

                        # CAR POSITION PLOT UPDATE
                        (car_x,car_y)=trackFrame_to_xyFrame(reshape(solHistory.z[j,lap,1,:],1,6),track)
                        update_prediction(car_plt,car_x,car_y,track)
                    end
                end

                # UPDATE THE CURVATURE PLOT
                curvature=curvature_prediction(reshape(solHistory.z[j,lap,:,:],size(solHistory.z[j,lap,:,:],3),size(solHistory.z[j,lap,:,:],4)),track)
                update_prediction(curvature_plt,solHistory.z[j,lap,:,1],curvature,track)
                ax_s_c[:set_xlim]([findmin(solHistory.z[j,lap,:,1])[1]-0.1, findmax(solHistory.z[j,lap,:,1])[1] + 0.1])
                ax_s_c[:set_title](statusHistory[j,lap])

                fig[:savefig]("$(homedir())/simulations/animations/lap$(lap)_iteration$(j).png", dpi=100)
                if select_flag
                    fig_select[:savefig]("$(homedir())/simulations/animations/lap$(lap)_iteration$(j)_select.png", dpi=100)
                end
                println("lap$(lap)_iteration$(j)")
            end
        end
        lap += 1
        if lap>length(solHistory.cost)
            break
        end
    end
end
