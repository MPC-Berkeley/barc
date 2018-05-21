import numpy as np
import datetime

class PID:
    """Create the PID controller used for path following at constant speed
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, vt):
        """Initialization
        Arguments:
            vt: target velocity
        """
        self.vt = vt
        self.uPred = np.zeros([1,2])

        startTimer = datetime.datetime.now()
        endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
        self.solverTime = deltaTimer
        self.linearizationTime = deltaTimer
        self.feasible = 1

    def solve(self, x0):
        """Computes control action
        Arguments:
            x0: current state position
        """
        vt = self.vt
        Steering = -2.0 * x0[5] - 0.5 * x0[3]
        Accelera = 0.1 * 1.5 * (vt - x0[0])
        self.uPred[0, 0] = np.maximum(-0.6, np.minimum(Steering, 0.6))
        self.uPred[0, 1] = np.maximum(-0.5, np.minimum(Accelera, 0.5))