function find_idx(s::Float64,track)
    idx = Int(ceil(s/track.ds)+1)
    if idx > track.n_node
        idx -= track.n_node
    end
    return idx
end

function trackFrame_to_xyFrame(z_sol::Array{Float64,2},track)
    # Position sanity check
    n = size(z_sol,1)
    z_x = zeros(n)
    z_y = zeros(n)
    for i in 1:n
        z = z_sol[i,:]
        if z[1]>track.s
            z[1]-=track.s
        elseif z[1]<0
            z[1]+=track.s
        end
        ds=track.ds; s=z[1]; ey=z[2]; epsi=z[3]
        idx = find_idx(z[1],track)

        x_track=track.xy[idx,1]
        y_track=track.xy[idx,2]
        theta=track.theta[idx]
        x=x_track+ey*cos(theta+pi/2)
        y=y_track+ey*sin(theta+pi/2)

        z_x[i]=x
        z_y[i]=y
    end
    return z_x, z_y
end