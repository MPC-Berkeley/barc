# This function evaluates and returns the coefficients for constraints and cost which are used later in the MPC formulation
# Inputs are:
# oldTraj   -> contains information about previous trajectories and Inputs
# lapStatus -> contains information about the current lap
# mpcCoeff  -> contains information about previous coefficient results
# posInfo   -> contains information about the car's current position along the track
# mpcParams -> contains information about the MPC formulation (e.g. Q, R)
# stateIn   -> current state of the car
# inputIn   -> last input to the system

# structure of oldTrajectory: 1st dimension = state number, 2nd dimension = step number (time equiv.), 3rd dimennsion = lap number

function coeffConstraintCost(oldTraj::OldTrajectory, lapStatus::LapStatus, mpcCoeff::MpcCoeff, posInfo::PosInfo, mpcParams::MpcParams)
    # this computes the coefficients for the cost and constraints

    # Outputs: 
    # coeffConst
    # coeffCost
    mpcCoeff::MpcCoeff

    # Read Inputs
    #tic()
    oldTrajectory   = oldTraj.oldTraj           # [:,:,1] = 1st, [:,:,2] = 2nd
    oldInput        = oldTraj.oldInput

    lapNumber       = lapStatus.currentLap

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
    coeffConst      = zeros(Order+1,2,3)       # nz-1 beacuse no coeff for s

    # Select the old data
    oldS            = oldTrajectory[:,1,:]
    oldeY           = oldTrajectory[:,2,:]
    oldePsi         = oldTrajectory[:,3,:]
    oldV            = oldTrajectory[:,4,:]

    N_points        = size(oldTrajectory,1)     # second dimension = length

    if lapNumber > 1
        # tt=toq()
        # println("Loading parameters: $tt")
        # tic()
        
        # Compute the total s (current position along track)
        s_total = (s_start + s) % s_target

        # Compute the index
        DistS = ( s_total - oldS ).^2

        idx_s = findmin(DistS,1)[2]              # contains both indices for the closest distances for both oldS !!

        vec_range = (idx_s[1]:idx_s[1]+pLength,idx_s[2]:idx_s[2]+pLength)
        # if idx_s[1]+pLength > N_points || idx_s[2]+pLength > 2*N_points
        #     warn("Out of range!")
        #     println("vec_range = $vec_range")
        #     println("idx_s = $idx_s")
        #     println("s_total = $s_total")
        # end
        
        # Create the vectors used for the interpolation
        # **************** WHAT IF MINIMUM INDEX + PLENGTH IS LONGER THAN ENTIRE OLD TRAJECTORY ? *******************************
        # -> oldTrajectory is designed to go way beyond the "real" measured limit
        bS_Vector       = zeros(pLength+1,2)
        beY_Vector      = zeros(pLength+1,2)
        bePsi_Vector    = zeros(pLength+1,2)
        bV_Vector       = zeros(pLength+1,2)

        bS_Vector       = cat(2, oldS[vec_range[1]],    oldS[vec_range[2]])
        beY_Vector      = cat(2, oldeY[vec_range[1]],   oldeY[vec_range[2]])
        bePsi_Vector    = cat(2, oldePsi[vec_range[1]], oldePsi[vec_range[2]])
        bV_Vector       = cat(2, oldV[vec_range[1]],    oldV[vec_range[2]])

        # tt = toq()
        # println("2nd: $tt")
        # tic()
        println("************************************** COEFFICIENTS **************************************")
        println("idx_s[1]  = $(idx_s[1]), idx_s[2] = $(idx_s[2])")
        println("s_total   = $s_total")
        # println("bS_Vector[1] = $(bS_Vector[:,:,1]')")
        # These matrices (above) contain two vectors each (for both old trajectories), stored in the 3rd dimension
        
        # The states are parametrized with resprect to the curvilinear abscissa,
        # so we select the point used for the interpolation. Need to subtract an
        # offset to be coherent with the MPC formulation
        s_forinterpy   = bS_Vector - s_start
        if s_total - s_start < 0
            s_forinterpy += s_target
        end
        println("s_forinterpy[:,1,1]' = $(s_forinterpy[:,1,1]')")
        # Create the Matrices for the interpolation
        MatrixInterp = zeros(pLength+1,Order+1,2)

        for k = 0:Order
            MatrixInterp[:,Order+1-k,:] = s_forinterpy[:,:].^k
        end
        
        # Compute the coefficients
        CoefficientsFor_ey      = zeros(Order+1,2)
        CoefficientsFor_ePsi    = zeros(Order+1,2)
        CoefficientsFor_V       = zeros(Order+1,2)
        println("Calculating coefficients...")
        for i=1:2
            CoefficientsFor_ey[:,i]      = MatrixInterp[:,:,i]\beY_Vector[:,i]
            CoefficientsFor_ePsi[:,i]    = MatrixInterp[:,:,i]\bePsi_Vector[:,i]
            CoefficientsFor_V[:,i]       = MatrixInterp[:,:,i]\bV_Vector[:,i]
        end
        println("Done.")
        
        # Stack the coefficients
        coeffConst = zeros(Order+1,2,3)
        coeffConst[:,:,1]                       = CoefficientsFor_ey
        coeffConst[:,:,2]                       = CoefficientsFor_ePsi
        coeffConst[:,:,3]                       = CoefficientsFor_V
        #tt = toq()
        #println("3rd: $tt")
        #tic()
        # structure of coeffConst:
        # 1st dimension specifies state
        # 2nd dimension specifies steps
        # 3rd dimension not used
        # 4th dimension specifies lapNumber

        # Finished with calculating the constraint coefficients
        
        # Now compute the final cost coefficients

        # The Q-function contains for every point in the sampled safe set the minimum cost-to-go-value
        # These values are calculated for both old trajectories
        # The vector bQfunction_Vector contains the cost at each point in the interpolated area to reach the finish line
        # From this vector, polynomial coefficients coeffCost are calculated to approximate this cost
        for i=1:2
            Qfunction  = zeros(N_points,1)
            IndexBezierS = idx_s[i] - (i-1)*N_points        # IndexBezierS is the index specifying the current position
            idx_s_target = find(oldS[:,i].>s_target)[1]
            dist_to_s_target = (idx_s_target - IndexBezierS)
            #dist_to_s_target = dist_to_s_target + 30

            bQfunction_Vector = zeros(pLength+1,1)
            # Select the part needed for the interpolation
            #bQfunction_Vector                   = Qfunction[IndexBezierS:IndexBezierS+pLength]
            qLength = min(dist_to_s_target,pLength+1)
            #println(bQfunction_Vector)
            bQfunction_Vector                   = zeros(pLength+1,1)
            bQfunction_Vector[1:qLength]        = (dist_to_s_target:-1:dist_to_s_target-qLength+1)*0.1

            #bQfunction_Vector                   = collect((dist_to_s_target:-1:dist_to_s_target-pLength))*0.1
            #println("length = $(length(bQfunction_Vector)), $(pLength+1)")
            #println(bQfunction_Vector)
            #readline()

            # Compute coefficient for the cost
            coeffCost[:,i]      = MatrixInterp[:,:,i]\bQfunction_Vector
            # if maximum(coeffCost) > 1e4
            #     warn("Large coefficients in cost, might cause numerical problems.")
            #     s = s_forinterpy[:,1,i]
            #     interp = [s.^5 s.^4 s.^3 s.^2 s.^1 s.^0] * coeffCost[:,1,i]
            #     println(coeffCost)
            #     plot(s,bQfunction_Vector,"-o",s,interp)
            #     grid()
            #     readline()
            # end

        end
        #tt = toq()
        #println("4th: $tt")

        # if maximum(coeffConst) > 1e4
        #     warn("Large coefficients in constraints, might cause numerical problems.")
        #     println(coeffConst)
        #     subplot(311)
        #     plot(s_forinterpy[:,:,1],beY_Vector[:,:,1],"-o")
        #     subplot(312)
        #     plot(s_forinterpy[:,:,1],bePsi_Vector[:,:,1],"-o")
        #     subplot(313)
        #     plot(s_forinterpy[:,:,1],bV_Vector[:,:,1],"-o")
        #     grid()
        #     readline()
        # end
        

    else        # if it is the first lap
        coeffCost            = zeros(Order+1,2)
        coeffConst           = zeros(Order+1,2,3) # nz-1 because no coeff for s
    end
    mpcCoeff = MpcCoeff(coeffCost, coeffConst, mpcCoeff.order, mpcCoeff.pLength)
    return mpcCoeff
    #return MpcCoeff(coeffCost, coeffConst, mpcCoeff.order, mpcCoeff.pLength)
end
