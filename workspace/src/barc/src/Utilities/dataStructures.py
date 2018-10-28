import numpy as np
import rospy
from barc.msg import pos_info, ECU, prediction, SafeSetGlob


class LMPCprediction():
    """Object collecting the predictions and SS at eath time step
    """
    def __init__(self, N, n, d, TimeLMPC, numSS_Points, Laps):
        """
        Initialization:
            N: horizon length
            n, d: input and state dimensions
            TimeLMPC: maximum simulation time length [s]
            num_SSpoints: number used to buils SS at each time step
        """
        self.oneStepPredictionError = np.zeros((n, TimeLMPC, Laps))
        self.PredictedStates        = np.zeros((N+1, n, TimeLMPC, Laps))
        self.PredictedInputs        = np.zeros((N, d, TimeLMPC, Laps))

        self.SSused   = np.zeros((n , numSS_Points, TimeLMPC, Laps))
        self.Qfunused = np.zeros((numSS_Points, TimeLMPC, Laps))

class EstimatorData(object):
    """Data from estimator"""
    def __init__(self):
        """Subscriber to estimator"""
        rospy.Subscriber("pos_info", pos_info, self.estimator_callback)
        self.CurrentState = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.CurrentAppliedSteeringInput = 0.0
    
    def estimator_callback(self, msg):
        """
        Unpack the messages from the estimator
        """
        self.CurrentState = [msg.v_x, msg.v_y, msg.psiDot, msg.psi, msg.x, msg.y]
        self.CurrentAppliedSteeringInput = msg.u_df

class ClosedLoopDataObj():
    """Object collecting closed loop data points
    Attributes:
        updateInitialConditions: function which updates initial conditions and clear the memory
    """
    def __init__(self, dt, Time, v0):
        """Initialization
        Arguments:
            dt: discretization time
            Time: maximum time [s] which can be recorded
            v0: velocity initial condition
        """
        self.dt = dt
        self.Points = int(Time / dt)  # Number of points in the simulation
        self.measSteering = np.zeros((self.Points, 1))  # Initialize the input vector
        self.u = np.zeros((self.Points, 2))  # Initialize the input vector
        self.x = np.zeros((self.Points + 1, 6))  # Initialize state vector (In curvilinear abscissas)
        self.x_glob = np.zeros((self.Points + 1, 6))  # Initialize the state vector in absolute reference frame
        self.solverTime = np.zeros((self.Points + 1, 1))  # Initialize state vector (In curvilinear abscissas)
        self.sysIDTime  = np.zeros((self.Points + 1, 1))  # Initialize state vector (In curvilinear abscissas)
        self.contrTime  = np.zeros((self.Points + 1, 1))  # Initialize state vector (In curvilinear abscissas)
        self.SimTime = -1
        self.x[0,0] = v0
        self.x_glob[0,0] = v0
        self.CurrentState = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def updateInitialConditions(self, x, x_glob):
        """Clears memory and resets initial condition
        x: initial condition is the curvilinear reference frame
        x_glob: initial condition in the inertial reference frame
        """
        self.x[0, :] = x
        self.x_glob[0, :] = x_glob

        self.x[1:, :] = np.zeros((self.x.shape[0]-1, 6))
        self.x_glob[1:, :] = np.zeros((self.x.shape[0]-1, 6))
        self.SimTime = -1

    def addMeasurement(self, xMeasuredGlob, xMeasuredLoc, uApplied, solverTime, sysIDTime, contrTime, measSteering):
        """Add point to the object ClosedLoopData
        xMeasuredGlob: measured state in the inerial reference frame
        xMeasuredLoc: measured state in the curvilinear reference frame
        uApplied: input applied to the system
        """
        self.SimTime = self.SimTime + 1
        self.x[self.SimTime, :]      = xMeasuredLoc
        self.x_glob[self.SimTime, :] = xMeasuredGlob
        self.u[self.SimTime, :]      = uApplied
        self.solverTime[self.SimTime, :]  = solverTime
        self.sysIDTime[self.SimTime, :]   = sysIDTime
        self.contrTime[self.SimTime, :]   = contrTime
        self.measSteering[self.SimTime, :]      = measSteering

class AvoidanceTrajectory():
    def __init__(self):
        rospy.Subscriber("other_agent_predicitons", prediction, self.traj_callback)
        self.s = []
        self.ey = []
        self.epsi = []
        self.length = 0.3
        self.width = 0.15
        self.safeZone = 2
        self.trackWidth = 1
        self.N = N

    def setHorizon(self, N): 
        self.N = N
        return self

    def setCarSize(self, length, width):
        self.length = length
        self.width = width
        return self 

    def setSafeZone(self, carWidths): 
        self.safeZone = carWidths
        return self

    def setTrackWidth(self, trackWidth):
        self.trackWidth = trackWidth
        return self

    def traj_callback(self, traj_msg): 
        self.s = traj_msg.s
        self.ey = traj_msg.ey
        self.epsi = traj_msg.epsi

    def computeConstraints(self, s, ey, epsi):
        bx = []
        if self.s[0] > s and np.norm(self.s[0] - s) <= self.safeZone*self.length: 
            # add collision constraints
            for i in range(N):
                ey = self.ey[i]
                if ey >= 0: 
                    upBound = ey - self.width/2
                    lowBound = self.trackWidth
                else: 
                    upBound = self.trackWidth
                    lowBound = -1.0 * (ey + self.width/2)
                bx.append(np.array([[upBound], [lowBound]]))
        else: 
            # add box track constraints
            for i in range(N):
                bx.append(np.array([[self.trackWidth], [self.trackWidth]]))

        bx = np.vstack(bx)
        return bx


