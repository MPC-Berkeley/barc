function InitializeParameters(mpcParams::MpcParams,mpcParams_pF::MpcParams,trackCoeff::TrackCoeff,modelParams::ModelParams,
                                posInfo::PosInfo,mpcCoeff::MpcCoeff,lapStatus::LapStatus,buffersize::Int64)
    mpcParams.N                 = 12
    mpcParams.Q                 = [5.0,0.0,0.0,1.0,10.0,0.0]   # Q (only for path following mode)
    mpcParams.vPathFollowing    = 0.9                           # reference speed for first lap of path following
    mpcParams.Q_term            = 1.0*[20.0,1.0,10.0,20.0,50.0]   # weights for terminal constraints (LMPC, for xDot,yDot,psiDot,ePsi,eY)
    mpcParams.R                 = 0*[10.0,10.0]                 # put weights on a and d_f
    mpcParams.QderivZ           = 1.0*[1,1,1,1,1,0]             # cost matrix for derivative cost of states
    mpcParams.QderivU           = 1.0*[5.0,100.0] #NOTE Set this to [5.0, 0/40.0]              # cost matrix for derivative cost of inputs
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
