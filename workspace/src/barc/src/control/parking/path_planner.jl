#!/usr/bin/env julia

using JuMP, Ipopt
using YAML
using JLD, NPZ

println("loading utility functions....")
include("modules/setup.jl")

println("loading configurations ....")
include("obstacle-configuration.jl")
include("vehicle-configuration.jl")
include("modules/mpcPathFollowing.jl")

home            = ENV["HOME"]
param_file 		= string(home,"/barc/workspace/src/barc/launch/control/parking.yaml")
save_path 		= string(home,"/barc/workspace/src/barc/src/control/parking/data")

data = YAML.load(open(param_file))

s0 	= data["initial_state"]
sF 	= data["final_state"]
L 	= data["vehicle_length"]

println("calling hybrid a* to get initial guess for warm start ...")
rx, ry, ryaw = hybrid_a_star.calc_hybrid_astar_path(s0[1], s0[2], s0[3], sF[1], sF[2], sF[3], ox, oy, hybrid_a_star.XY_GRID_RESOLUTION, hybrid_a_star.YAW_GRID_RESOLUTION, hybrid_a_star.OB_MAP_RESOLUTION)

println("smoothing hybrid a* solution ...")
xWS, uWS, N = getSmoothProfile(rx,ry,ryaw,Ts,sampleN,motionStep)

println("computing H-representation for obstacle polytopes ....")
AOb, bOb = obstHrep(nOb, vOb, lOb)

println("solving finite time optimal control problem ...")
sp10, up10, scaleTime10, exitflag10, time10, lp10, np10 = ParkingSignedDist(s0,sF,N,Ts,L,ego,XYbounds,nOb,vObMPC,AOb,bOb,fixTime,xWS,uWS)

println("interpolating solution ....")
tDomOpt = unshift!(cumsum(scaleTime10*Ts),0)
pop!(tDomOpt)
tDomSim 	= 0:0.1:round(tDomOpt[end-1],1)
fsp10 		= intp.interp1d(tDomOpt, sp10)
fup10 		= intp.interp1d(tDomOpt[1:end-1], up10)
sp10Ref 	= fsp10(tDomSim)
up10Ref 	= fup10(tDomSim)
dp10Ref 	=  cumsum( sqrt.( diff(sp10Ref[1,:]).^2 .+ diff(sp10Ref[2,:]).^2 ) )
unshift!(dp10Ref,0)

println("saving solution ...")
save("$save_path/parkPath.jld","sp10",sp10,"up10",up10,"scaleTime10",scaleTime10,"sp10Ref",sp10Ref,"up10Ref",up10Ref,"dp10Ref",dp10Ref)
npzwrite("$save_path/npz/sp10.npz", sp10) 
npzwrite("$save_path/npz/up10.npz", up10) 
npzwrite("$save_path/npz/scaleTime10.npz", scaleTime10) 
npzwrite("$save_path/npz/sp10Ref.npz", sp10Ref) 
npzwrite("$save_path/npz/up10Ref.npz", up10Ref) 
npzwrite("$save_path/npz/dp10Ref.npz", dp10Ref) 

println("plotting solution path")
for j = 1 : nObPlot
	for k = 1 : vObPlot[j]
		plot([lObPlot[j][k][1],lObPlot[j][k+1][1]] , [lObPlot[j][k][2],lObPlot[j][k+1][2]] ,"k")
	end
end
plot(sp10[1,:],sp10[2,:], "-b",  label="H-OBCA")
plot(s0[1],s0[2],"ob")
plot(sF[1],sF[2],"ob")
legend()
axis("equal")
