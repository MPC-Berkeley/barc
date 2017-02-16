
function coeffConstraintCost(mpcTraj::MpcTrajectory, mpcCoeff::MpcCoeff, posInfo::PosInfo, mpcParams::MpcParams,lapStatus::LapStatus)
    # this computes the coefficients for the terminal cost and terminal constraints

    # Read Inputs
    s               = posInfo.s
    s_target        = posInfo.s_target

    # Parameters
    Order           = mpcCoeff.order                # interpolation order for cost and constraints
    pLength         = mpcCoeff.pLength              # interpolation length for polynomials
    delay_df        = mpcParams.delay_df

    selected_laps = zeros(Int64,2)
    selected_laps[1] = lapStatus.currentLap-1                                   # use previous lap
     selected_laps[2] = lapStatus.currentLap-2                                   # and the one before
     if lapStatus.currentLap >= 4
         selected_laps[2] = indmin(mpcTraj.cost[1,1,2:lapStatus.currentLap-2])+1      # and the best from all previous laps
     end

    xfRange = zeros(2,2)
    # do coeffcient calculation once for previous lap and once for best lap
    for kkk = 1:2
        lapNum = selected_laps[kkk]
        data_end = mpcTraj.count[lapNum]-1
        oldS            = mpcTraj.closedLoopSEY[1:data_end,1,lapNum]
        oldeY           = mpcTraj.closedLoopSEY[1:data_end,2,lapNum]
        oldePsi         = mpcTraj.closedLoopSEY[1:data_end,3,lapNum]
        oldV            = mpcTraj.closedLoopSEY[1:data_end,4,lapNum]
        oldRho          = mpcTraj.closedLoopSEY[1:data_end,5,lapNum]
        oldePsiRef      = mpcTraj.closedLoopSEY[1:data_end,6,lapNum]

        # Compute the index
        DistS = ( s - oldS ).^2
        idx_s = Int64(findmin(DistS)[2])
        
        vec_range = idx_s:idx_s+pLength
        if vec_range[end] > data_end
            lll = data_end-idx_s-1
            vec_range = idx_s:idx_s+lll
        end
        bS_Vector = oldS[vec_range]
        xfRange[1,kkk] = vec_range[1]
        xfRange[2,kkk] = vec_range[end]

        # Create the Matrices for the interpolation
        MatrixInterp = zeros(length(bS_Vector),Order+1)

        for k = 0:Order
            MatrixInterp[:,Order+1-k] = bS_Vector.^k
        end
        
        # Compute the coefficients
        mpcCoeff.coeffConst[:,kkk,1]    = MatrixInterp[:,:]\oldeY[vec_range]
        mpcCoeff.coeffConst[:,kkk,2]    = MatrixInterp[:,:]\oldePsi[vec_range]
        mpcCoeff.coeffConst[:,kkk,3]    = MatrixInterp[:,:]\oldV[vec_range]
        mpcCoeff.coeffConst[:,kkk,4]    = MatrixInterp[:,:]\oldRho[vec_range]
        mpcCoeff.coeffConst[:,kkk,5]    = MatrixInterp[:,:]\oldePsiRef[vec_range]

        # Finished with calculating the constraint coefficients
        
        # Now compute the final cost coefficients
        # The Q-function contains for every point in the sampled safe set the minimum cost-to-go-value
        costVector = mpcTraj.cost[vec_range,1,lapNum]                               # decreases in equal steps
        mpcCoeff.coeffCost[:,kkk] = MatrixInterp[:,:]\costVector           # interpolate this vector with the given s
    end
    return (xfRange,selected_laps)
end

