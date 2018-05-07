using JLD
using PyPlot

# PLOT SETTING
run_time 	= Dates.format(now(),"yyyy-mm-dd-H:M")
data 		= load("LMPC-$(run_time).jld")

# DATA READING
log_cvx 	= data["log_cvx"]
log_cvy 	= data["log_cvy"]
log_cpsi 	= data["log_cpsi"]
oldSS 		= data["oldSS"]
solHistory 	= data["solHistory"]
# oldTraj 	= data["oldTraj"]
# track 		= data["track"]

if ARGS[1] == "theta"
	figure()
	plot(log_cvx[1:oldSS.oldCost[lapNum],1,1,lapNum],label="cvx1")
	# # plot(1+0.1*log_cvx[1:oldSS.oldCost[lapNum],1,1,lapNum],label="cvx1")
	plot(log_cvx[1:oldSS.oldCost[lapNum],1,2,lapNum],label="cvx2")
	plot(log_cvx[1:oldSS.oldCost[lapNum],1,3,lapNum],label="cvx3")
	# plot(oldSS.oldSS[1:oldSS.oldCost[lapNum],4,lapNum],"--",label="vx")
	legend()

	figure()
	plot(log_cvy[1:oldSS.oldCost[lapNum],1,1,lapNum],label="cvy1")
	# plot(1.5+0.5*log_cvy[1:oldSS.oldCost[lapNum],1,1,lapNum],label="cvy1")
	plot(log_cvy[1:oldSS.oldCost[lapNum],1,2,lapNum],label="cvy2")
	plot(log_cvy[1:oldSS.oldCost[lapNum],1,3,lapNum],label="cvy3")
	plot(log_cvy[1:oldSS.oldCost[lapNum],1,4,lapNum],label="cvy4")
	# plot(oldSS.oldSS[1:oldSS.oldCost[lapNum],4,lapNum],"--",label="vx")
	legend()

	figure()
	plot(log_cpsi[1:oldSS.oldCost[lapNum],1,1,lapNum],label="cpsi1")
	plot(log_cpsi[1:oldSS.oldCost[lapNum],1,2,lapNum],label="cpsi2")
	# plot(1+0.1*log_cpsi[1:oldSS.oldCost[lapNum],1,2,lapNum],label="cpsi2")
	plot(log_cpsi[1:oldSS.oldCost[lapNum],1,3,lapNum],label="cpsi3")
	# plot(oldSS.oldSS[1:oldSS.oldCost[lapNum],4,lapNum],"--",label="vx")
	legend()
elseif ARGS[1] == "trajectory"
	figure()

end
show()