function computeCostLap(z,s_target)

    f = find(z[:,1].>=s_target)
    cost = 100000
    if size(f,1)>0
        cost = f[1]
    end
    return cost
end