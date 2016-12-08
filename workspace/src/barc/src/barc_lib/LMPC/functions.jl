# Important information about oldTraj:
# ======================================
# oldTraj.oldTraj contains all state information of one lap. It is structured like follows:
# The first <prebuf> values are the end of one lap before the next lap starts (necessary for inter-lap-system-ID)
# The last value of <prebuf> ends with the last value *before* the finish line (s < s_target)
# The s-values within <prebuf> need to be below zero (-> before the finish line). Otherwise they can be mistaken as the end of the current (not previous) lap.
# After <prebuf> follow the recorded states of the trajectory (as many as there were during one lap)
# The number of recorded states during one trajectory (0 <= s < s_target) is equal to <oldCost>.
# After the recorded trajectory, the rest of the vector (until <buffersize>) is filled up with constant values

function saveOldTraj(oldTraj::OldTrajectory,zCurr::Array{Float64},uCurr::Array{Float64},lapStatus::LapStatus,posInfo::PosInfo,buffersize::Int64)
                println("Starting function")
                i               = lapStatus.currentIt-1         # i = number of points for 0 <= s < s_target (= cost of this lap)
                prebuf          = oldTraj.prebuf                # so many points of the end of the previous old traj will be attached to the beginning
                zCurr_export    = zeros(buffersize,6)
                uCurr_export    = zeros(buffersize,2)

                println("Creating exports")
                zCurr_export    = cat(1,oldTraj.oldTraj[oldTraj.oldCost[1]+1:oldTraj.oldCost[1]+prebuf,:,1],
                                        zCurr[1:i,:], NaN*ones(buffersize-i-prebuf,6))#[ones(buffersize-i-prebuf,1)*zCurr[i,1:5] zCurr[i,6]+collect(1:buffersize-i-prebuf)*dt*zCurr[i,1]])
                uCurr_export    = cat(1,oldTraj.oldInput[oldTraj.oldCost[1]+1:oldTraj.oldCost[1]+prebuf,:,1],
                                        uCurr[1:i,:], NaN*ones(buffersize-i-prebuf,2))#zeros(buffersize-i-prebuf,2))

                zCurr_export[1:prebuf,6] -= posInfo.s_target       # make the prebuf-values below zero
                costLap                   = i                      # the cost of the current lap is the time it took to reach the finish line
                println("Saving")
                # Save all data in oldTrajectory:
                if lapStatus.currentLap <= 2                        # if it's the first or second lap
                    oldTraj.oldTraj[:,:,1]  = zCurr_export          # ... just save everything
                    oldTraj.oldInput[:,:,1] = uCurr_export
                    oldTraj.oldTraj[:,:,2]  = zCurr_export
                    oldTraj.oldInput[:,:,2] = uCurr_export
                    oldTraj.oldCost = [costLap,costLap]
                else                                                # idea: always copy the new trajectory in the first array!
                    if oldTraj.oldCost[1] < oldTraj.oldCost[2]      # if the first old traj is better than the second
                        oldTraj.oldTraj[:,:,2]  = oldTraj.oldTraj[:,:,1]    # ... copy the first in the second
                        oldTraj.oldInput[:,:,2] = oldTraj.oldInput[:,:,1]   # ... same for the input
                        oldTraj.oldCost[2] = oldTraj.oldCost[1]
                    end
                    oldTraj.oldTraj[:,:,1]  = zCurr_export                 # ... and write the new traj in the first
                    oldTraj.oldInput[:,:,1] = uCurr_export
                    oldTraj.oldCost[1] = costLap
                end
end

function InitializeParameters(mpcParams::MpcParams,mpcParams_pF::MpcParams,trackCoeff::TrackCoeff,modelParams::ModelParams,
                                posInfo::PosInfo,oldTraj::OldTrajectory,mpcCoeff::MpcCoeff,lapStatus::LapStatus,buffersize::Int64)
    mpcParams.N                 = 12
    mpcParams.Q                 = [5.0,0.0,0.0,1.0,10.0,0.0]   # Q (only for path following mode)
    mpcParams.vPathFollowing    = 0.8                           # reference speed for first lap of path following
    mpcParams.Q_term            = 1.0*[20.0,1.0,10.0,20.0,50.0]   # weights for terminal constraints (LMPC, for xDot,yDot,psiDot,ePsi,eY)
    mpcParams.R                 = 0*[10.0,10.0]                 # put weights on a and d_f
    mpcParams.QderivZ           = 1.0*[1,1,1,1,1,0]             # cost matrix for derivative cost of states
    mpcParams.QderivU           = 1.0*[5.0,40.0]                # cost matrix for derivative cost of inputs
    mpcParams.Q_term_cost       = 2.0                         # scaling of Q-function
    mpcParams.delay_df          = 3                             # steering delay
    mpcParams.delay_a           = 1                             # acceleration delay

    mpcParams_pF.N              = 15
    mpcParams_pF.Q              = [0.0,50.0,0.1,10.0]
    mpcParams_pF.R              = 0*[1.0,1.0]               # put weights on a and d_f
    mpcParams_pF.QderivZ        = 0.0*[0,0,0.1,0]           # cost matrix for derivative cost of states
    mpcParams_pF.QderivU        = 1.0*[10,10]                # cost matrix for derivative cost of inputs
    mpcParams_pF.vPathFollowing = 0.9                       # reference speed for first lap of path following
    mpcParams_pF.delay_df       = 3                         # steering delay (number of steps)
    mpcParams_pF.delay_a        = 1                         # acceleration delay

    trackCoeff.nPolyCurvature   = 8                         # 4th order polynomial for curvature approximation
    trackCoeff.coeffCurvature   = zeros(trackCoeff.nPolyCurvature+1)         # polynomial coefficients for curvature approximation (zeros for straight line)
    trackCoeff.width            = 0.6                       # width of the track (0.5m)

    modelParams.l_A             = 0.125
    modelParams.l_B             = 0.125
    modelParams.dt              = 0.1                   # sampling time, also controls the control loop, affects delay_df and Qderiv
    modelParams.m               = 1.98
    modelParams.I_z             = 0.03
    modelParams.c_f             = 0.5                   # friction coefficient: xDot = - c_f*xDot (aerodynamic+tire)

    posInfo.s_target            = 5.0

    oldTraj.oldTraj             = NaN*ones(buffersize,7,30)
    oldTraj.oldInput            = zeros(buffersize,2,30)
    oldTraj.oldTimes            = NaN*ones(buffersize,30)
    oldTraj.count               = ones(30)*2
    oldTraj.oldCost             = ones(Int64,30)                   # dummies for initialization
    oldTraj.prebuf              = 30
    oldTraj.postbuf             = 30
    oldTraj.idx_start           = zeros(30)
    oldTraj.idx_end             = zeros(30)

    mpcCoeff.order              = 5
    mpcCoeff.coeffCost          = zeros(mpcCoeff.order+1,2)
    mpcCoeff.coeffConst         = zeros(mpcCoeff.order+1,2,5)
    mpcCoeff.pLength            = 5*2*mpcParams.N        # small values here may lead to numerical problems since the functions are only approximated in a short horizon
    mpcCoeff.c_Vx               = zeros(3)
    mpcCoeff.c_Vy               = zeros(4)
    mpcCoeff.c_Psi              = zeros(3)

    lapStatus.currentLap        = 1         # initialize lap number
    lapStatus.currentIt         = 0         # current iteration in lap
end

# Use this function to smooth data (moving average)
function smooth(x,n)
    y = zeros(size(x))
    for i=1:size(x,1)
        start = max(1,i-n)
        fin = min(size(x,1),start + 2*n)
        y[i,:] = mean(x[start:fin,:],1)
    end
    return y
end
