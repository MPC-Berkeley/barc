# This function evaluates and returns the coefficients for constraints and cost which are used later in the MPC formulation
# Inputs are:
# oldTraj   -> contains information about previous trajectories and Inputs
# mpcCoeff  -> contains information about previous coefficient results
# posInfo   -> contains information about the car's current position along the track
# mpcParams -> contains information about the MPC formulation (e.g. Q, R)
# stateIn   -> current state of the car
# inputIn   -> last input to the system

# structure of oldTrajectory: 1st dimension = state number, 2nd dimension = step number (time equiv.), 3rd dimennsion = lap number

function coeffConstraintCost(oldTraj::OldTrajectory, mpcCoeff::MpcCoeff, posInfo::PosInfo, mpcParams::MpcParams)
    # this computes the coefficients for the cost and constraints

    # Outputs: 
    # coeffConst
    # coeffCost

    # Read Inputs
    s_start         = posInfo.s_start
    s               = posInfo.s
    s_target        = posInfo.s_target


    # Parameters
    N               = mpcParams.N
    nz              = mpcParams.nz
    R               = mpcParams.R
    Order           = mpcCoeff.order                # interpolation order for cost and constraints

    pLength         = mpcCoeff.pLength              # interpolation length for polynomials

    coeffCost       = zeros(Order+1,2)            # polynomial coefficients for cost
    coeffConst      = zeros(Order+1,2,3)          # nz-1 beacuse no coeff for s

    # Select the old data
    oldS            = oldTraj.oldTraj[:,1,:]::Array{Float64,3}
    oldeY           = oldTraj.oldTraj[:,2,:]::Array{Float64,3}
    oldePsi         = oldTraj.oldTraj[:,3,:]::Array{Float64,3}
    oldV            = oldTraj.oldTraj[:,4,:]::Array{Float64,3}

    N_points        = size(oldTraj.oldTraj,1)     # second dimension = length

    s_total::Float64        # initialize
    DistS::Array{Float64}   # initialize
    idx_s::Array{Int64}     # initialize
    idx_s_target        = 0
    dist_to_s_target    = 0
    qLength             = 0
    vec_range::Tuple{UnitRange{Int64},UnitRange{Int64}}
    bS_Vector::Array{Float64}
    s_forinterpy::Array{Float64}

    # Compute the total s (current position along track)
    s_total = (s_start + s) % s_target

    # Compute the index
    DistS = ( s_total - oldS ).^2

    idx_s = findmin(DistS,1)[2]              # contains both indices for the closest distances for both oldS !!

    vec_range = (idx_s[1]:idx_s[1]+pLength,idx_s[2]:idx_s[2]+pLength)

    # Create the vectors used for the interpolation
    bS_Vector       = zeros(pLength+1,2)
    for i=1:pLength+1
        bS_Vector[i,1] = oldS[vec_range[1][i]]
        bS_Vector[i,2] = oldS[vec_range[2][i]]
    end
    # bS_Vector       = cat(2, oldS[vec_range[1]],    oldS[vec_range[2]])

    # println("************************************** COEFFICIENTS **************************************")
    # println("idx_s[1]  = $(idx_s[1]), idx_s[2] = $(idx_s[2])")
    # println("s_total   = $s_total")
    # println("bS_Vector[1] = $(bS_Vector[:,:,1]')")
    # These matrices (above) contain two vectors each (for both old trajectories), stored in the 3rd dimension
    
    # The states are parametrized with resprect to the curvilinear abscissa,
    # so we select the point used for the interpolation. Need to subtract an
    # offset to be coherent with the MPC formulation
    s_forinterpy   = bS_Vector - s_start
    if s_total - s_start < 0
        s_forinterpy += s_target
    end
    # println("s_forinterpy[:,1,1]' = $(s_forinterpy[:,1,1]')")
    # Create the Matrices for the interpolation
    MatrixInterp = zeros(pLength+1,Order+1,2)

    for k = 0:Order
        MatrixInterp[:,Order+1-k,:] = s_forinterpy[:,:].^k
    end
    
    # Compute the coefficients
    coeffConst = zeros(Order+1,2,3)
    for i=1:2
        coeffConst[:,i,1]    = MatrixInterp[:,:,i]\oldeY[vec_range[i]]
        coeffConst[:,i,2]    = MatrixInterp[:,:,i]\oldePsi[vec_range[i]]
        coeffConst[:,i,3]    = MatrixInterp[:,:,i]\oldV[vec_range[i]]
    end

    # Finished with calculating the constraint coefficients
    
    # Now compute the final cost coefficients

    # The Q-function contains for every point in the sampled safe set the minimum cost-to-go-value
    # These values are calculated for both old trajectories
    # The vector bQfunction_Vector contains the cost at each point in the interpolated area to reach the finish line
    # From this vector, polynomial coefficients coeffCost are calculated to approximate this cost
    for i=1:2
        Qfunction           = zeros(N_points,1)
        IndexBezierS        = idx_s[i] - (i-1)*N_points        # IndexBezierS is the index specifying the current position
        idx_s_target        = find(oldS[:,i].>s_target)[1]
        dist_to_s_target    = (idx_s_target - IndexBezierS)
        dist_to_s_target    = dist_to_s_target + 30

        bQfunction_Vector   = zeros(pLength+1,1)
        # Select the part needed for the interpolation
        #bQfunction_Vector                   = Qfunction[IndexBezierS:IndexBezierS+pLength]
        qLength             = min(dist_to_s_target,pLength+1)
        #println(bQfunction_Vector)
        bQfunction_Vector                   = zeros(pLength+1,1)
        bQfunction_Vector[1:qLength]        = (dist_to_s_target:-1:dist_to_s_target-qLength+1)*0.1

        #bQfunction_Vector                   = collect((dist_to_s_target:-1:dist_to_s_target-pLength))*0.1
        #println("length = $(length(bQfunction_Vector)), $(pLength+1)")
        #println(bQfunction_Vector)
        #readline()

        # Compute coefficient for the cost
        coeffCost[:,i]      = MatrixInterp[:,:,i]\bQfunction_Vector

    end

    mpcCoeff.coeffCost  = coeffCost
    mpcCoeff.coeffConst = coeffConst
    
    nothing
end
