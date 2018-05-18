import numpy as np
import os
import matplotlib.pyplot as plt

homedir = os.path.expanduser("~")
pathSave = os.path.join(homedir,"barc_debugging/estimator.npz")
npzfile = np.load(pathSave)

psi_drift_est_his 	=npzfile["psi_drift_est_his"]
psi_drift_est_2_his	=npzfile["psi_drift_est_2_his"]
psi_est_his 		=npzfile["psi_est_his"]
psi_est_2_his 		=npzfile["psi_est_2_his"]

fig = plt.figure("plotting")
ax1 = fig.add_subplot(2,1,1)
ax1.plot(psi_drift_est_his, label="psi_drift_1")
ax1.plot(psi_drift_est_2_his,label="psi_drift_2")
ax1.legend()

ax2 = fig.add_subplot(2,1,2)
ax2.plot(psi_est_his, label="psi_est_1")
ax2.plot(psi_est_2_his,label="psi_est_2")
ax2.legend()

plt.show()