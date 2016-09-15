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

function coeffConstraintCost(oldTraj, lapStatus, mpcCoeff, posInfo, mpcParams)
    # this computes the coefficients for the cost and constraints

    # Outputs: 
    # coeffConst
    # coeffCost

    # Read Inputs
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

    coeffCost       = zeros(Order+1,1,2)            # polynomial coefficients for cost
    coeffConst      = zeros(nz-1,Order+1,1,2)       # nz-1 beacuse no coeff for s

    # Select the old data
    oldS            = oldTrajectory[1,:,:]
    oldeY           = oldTrajectory[2,:,:]
    oldePsi         = oldTrajectory[3,:,:]
    oldV            = oldTrajectory[4,:,:]

    N_points        = size(oldTrajectory,2)     # second dimension = length

    if lapNumber > 1
        deltaOld = oldInput[1,:,:]
        aOld     = oldInput[2,:,:]
        
        # Compute the total s (current position along track)
        s_total = s_start + s

        # Compute the index
        DistS = ( s_total - oldS ).^2

        idx_s = findmin(DistS,2)[2]              # contains both indices for the closest distances for both oldS !!

        vec_range = (idx_s[1]:min(idx_s[1]+pLength,N_points),idx_s[2]:min(idx_s[2]+pLength,N_points*2))
        if idx_s[1]+pLength > N_points || idx_s[2]+pLength > 2*N_points
            warn("Out of range!")
            println("vec_range = $vec_range")
            println("idx_s = $idx_s")
            println("s_total = $s_total")
        end
        
        # Create the vectors used for the interpolation
        # **************** WHAT IF MINIMUM INDEX + PLENGTH IS LONGER THAN ENTIRE OLD TRAJECTORY ? *******************************
        # -> oldTrajectory is designed to go way beyond the "real" measured limit
        bS_Vector       = cat(3, oldS[vec_range[1]],    oldS[vec_range[2]])
        beY_Vector      = cat(3, oldeY[vec_range[1]],   oldeY[vec_range[2]])
        bePsi_Vector    = cat(3, oldePsi[vec_range[1]], oldePsi[vec_range[2]])
        bV_Vector       = cat(3, oldV[vec_range[1]],    oldV[vec_range[2]])
        # These matrices (above) contain two vectors each (for both old trajectories), stored in the 3rd dimension
        
        # The states are parametrized with resprect to the curvilinear abscissa,
        # so we select the point used for the interpolation. Need to subtract an
        # offset to be coherent with the MPC formulation
        s_forinterpy   = bS_Vector - s_start
        # Create the Matrices for the interpolation
        MatrixInterp = zeros(pLength+1,Order+1,2)

        for k = 0:Order
            MatrixInterp[:,Order+1-k,:] = s_forinterpy[:,1,:].^k
        end
        
        # Compute the coefficients
        CoefficientsFor_ey      = zeros(Order+1,1,2)
        CoefficientsFor_ePsi    = zeros(Order+1,1,2)
        CoefficientsFor_V       = zeros(Order+1,1,2)
        for i=1:2
            CoefficientsFor_ey[:,:,i]      = MatrixInterp[:,:,i]\beY_Vector[:,:,i]
            CoefficientsFor_ePsi[:,:,i]    = MatrixInterp[:,:,i]\bePsi_Vector[:,:,i]
            CoefficientsFor_V[:,:,i]       = MatrixInterp[:,:,i]\bV_Vector[:,:,i]
        end
        
        # Stack the coefficients
        coeffConst = zeros(3,Order+1,1,2)
        coeffConst[1,:,:,:]                       = CoefficientsFor_ey
        coeffConst[2,:,:,:]                       = CoefficientsFor_ePsi
        coeffConst[3,:,:,:]                       = CoefficientsFor_V
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
            idx_s_target = find(oldS[1,:,i].>s_target)[1]
            dist_to_s_target = (idx_s_target - IndexBezierS)
            dist_to_s_target = dist_to_s_target + 30         # set distance to finish line a bit higher
            # for k=1:N_points
            #     # Start from the end ---> reverse the index
            #     indx = N_points-k+1

            #     # Here the if for the cost, minimize the distance to a straight line
            #     if oldS[1,indx,i] <= s_target
            #         QMatrix = 0.1
            #     else
            #         QMatrix = 0
            #     end

            #     # Here actually computing the cost
            #     if (indx >=  IndexBezierS)
            #         if k == 1
            #             # If last point --> No Input
            #             Qfunction[indx] = QMatrix         # [1] to transform 1-element-array to scalar
            #         else
            #             Qfunction[indx] = Qfunction[indx + 1] + QMatrix
            #         end
            #     end
            # end
            # NEEDS TO BE IMPROVED **************************
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
            coeffCost[:,1,i]      = MatrixInterp[:,:,i]\bQfunction_Vector
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
        println("First lap, so everything's zero..")
        coeffCost            = zeros(Order+1,1,2)
        coeffConst           = zeros(nz-1,Order+1,1,2) # nz-1 because no coeff for s
    end
    return MpcCoeff(coeffCost, coeffConst, mpcCoeff.order, mpcCoeff.pLength)
end
