import numpy as np
import pdb
import scipy.optimize as optimize
import scipy.spatial

def Regression(x, u, lamb):
    """Estimates linear system dynamics
    x, u: date used in the regression
    lamb: regularization coefficient
    """

    # Want to solve W^* = argmin sum_i ||W^T z_i - y_i ||_2^2 + lamb ||W||_F,
    # with z_i = [x_i u_i] and W \in R^{n + d} x n
    Y = x[2:x.shape[0], :]
    X = np.hstack((x[1:(x.shape[0] - 1), :], u[1:(x.shape[0] - 1), :]))

    Q = np.linalg.inv(np.dot(X.T, X) + lamb * np.eye(X.shape[1]))
    b = np.dot(X.T, Y)
    W = np.dot(Q, b)

    A = W.T[:, 0:6]
    B = W.T[:, 6:8]

    ErrorMatrix = np.dot(X, W) - Y
    ErrorMax = np.max(ErrorMatrix, axis=0)
    ErrorMin = np.min(ErrorMatrix, axis=0)
    Error = np.vstack((ErrorMax, ErrorMin))

    return A, B, Error

def Curvature(s, PointAndTangent):
    """curvature computation
    s: curvilinear abscissa at which the curvature has to be evaluated
    PointAndTangent: points and tangent vectors defining the map (these quantities are initialized in the map object)
    """
    TrackLength = PointAndTangent[-1,3]+PointAndTangent[-1,4]

    # In case on a lap after the first one
    while (s > TrackLength):
        s = s - TrackLength

    # Given s \in [0, TrackLength] compute the curvature
    # Compute the segment in which system is evolving
    index = np.all([[s >= PointAndTangent[:, 3]], [s < PointAndTangent[:, 3] + PointAndTangent[:, 4]]], axis=0)

    i = int(np.where(np.squeeze(index))[0])
    curvature = PointAndTangent[i, 5]

    return curvature

class ConvexSafeSet:
    def __init__(self, SS, Q_SS, lambdas, disturbance_covariance = 0.002*np.identity(6).astype(float), n=10):
        self.SS = SS
        self.Q_SS = Q_SS
        self.lambdas = lambdas
        self.disturbance_covariance = disturbance_covariance
        self.n = n
        self.mu = np.zeros(6)
        CS = scipy.spatial.ConvexHull(self.SS.T)
        self.Hcs =  CS.equations[:,:-1]
        self.hcs =  -CS.equations[:,-1]
        self.hcs = self.hcs.reshape((len(self.hcs), 1))
        self.inCS = lambda pt: np.all(np.dot(Hcs, pt) <= hcs)


    def get_lambdas(self, x):
        x=x.reshape((len(x),1))
        nStates = self.SS.shape[1]
        A = np.vstack((np.ones((1,nStates)), self.SS))
        b = np.vstack((1,x))
        result = optimize.linprog(self.Q_SS, A_eq=A, b_eq=b)
        return result.x

    def estimate_failprob(self, x):
        #lambdas = self.get_lambdas(x)
        lambdas = self.lambdas
        lambdas = np.roll(lambdas,1)
        lambdas[-1] = lambdas[-1] + lambdas[0]
        lambdas[0] = 0
        xhat = np.dot(self.SS, lambdas.reshape(len(lambdas),1))
        num_states = len(xhat)
        const = np.dot(self.Hcs, xhat)
        sigma = 0.0005
        phat = 0
        for _ in range(self.n):
            sample = sigma * np.random.randn(num_states,1) - sigma*np.dot(np.random.randn(num_states, self.SS.shape[1]), lambdas)# np.random.multivariate_normal(mean=np.zeros(num_states), cov= (1 + np.dot(lambdas, lambdas)) *self.disturbance_covariance)
            #sample_combination = np.hstack((1,-lambdas))
            #sample = np.dot(np.random.multivariate_normal(self.mu, self.disturbance_covariance, size = len(sample_combination)).T, sample_combination) 
            phat += np.all(np.dot(self.Hcs, sample) < self.hcs - const)

        return phat/np.float(self.n)





        


