function gpPrepKin(e::Array{Float64,1},feature_state::Array{Float64,2})
    num = size(feature_state,1)
    Z=zeros(num,num)
    for i = 1:num
        for j=1:num
            z = feature_state[i,4:6]-feature_state[j,4:6]
            Z[i,j] = 5*z[1]^2+z[2]^2+5*z[3]^2
        end
    end
    K = exp(-300.0*Z)

    return K\e
end
function gpFullKin(z::Array{Float64,2},u::Array{Float64,2},feature_state::Array{Float64,2},GP_prepare::Array{Float64,1})
    state = hcat(z,u)
    z = feature_state[:,4:6].-state[1,4:6]
    Z = 5*z[:,1].^2+z[:,2].^2+5*z[:,3].^2
    k = exp(-300.0*Z)
    GP_e = k'*GP_prepare
    return GP_e[1]
end

function gpPrepDyn(e::Array{Float64,1},zu::Array{Float64,2})
    num = size(zu,1)
    Z=zeros(num,num)
    for i = 1:num
        for j=1:num
            z = zu[i,4:8]-zu[j,4:8]
            Z[i,j] = (10*z[1]^2+10*z[2]^2+z[3]^2+z[4]^2+5*z[5]^2)
        end
    end
    K=exp(-20.0*Z)
    # println(K)
    # println(e)
    return K\e
end

function gpFullDyn(z::Array{Float64,2},u::Array{Float64,2},feature_state::Array{Float64,2},GP_prepare::Array{Float64,1})
    state = hcat(z,u)
    z = feature_state[:,4:8].-state[1,4:8]
    Z = 10*z[:,1].^2+10*z[:,2].^2+z[:,3].^2+z[:,4].^2+5*z[:,5].^2
    k = exp(-20.0*Z)
    GP_e = k'*GP_prepare
    return GP_e[1]
end