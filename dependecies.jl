#!/usr/bin/env julia

#=
    File name: dependencies.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#


# Check julia version
if string(VERSION)[1:3] != "0.4"
    error("Julia version 0.4.x is needed. Please install a different julia 
    	   version.")
end

# Install required packages
# Enable the use of python in julia
Pkg.add("PyCall")
# Enable loading and saving hdf5 files
Pkg.add("HDF5") 
# Enable loading and saving variables in JLD-format
Pkg.add("JLD")
Pkg.pin("JLD", v"0.6.10")
# Enable visualization of profiling data
Pkg.add("ProfileView")
# Add Ipopt interface to julia
Pkg.add("Ipopt")
# Add JuMP for mathematical optimization in julia
Pkg.add("JuMP")
# Enable plotting with matplotlib in julia
Pkg.add("PyPlot")
# Add functionalities for polynomials in julia
# TODO: Can probably remove this dependency
Pkg.add("Polynomials")
# Enable efficient calculation of convex hulls 
Pkg.add("QuickHull")
# Enable efficient calculation of distances
Pkg.add("Distances")
# Enable the use of ROS in julia
Pkg.add("RobotOS")
Pkg.pin("RobotOS", v"0.4.2")




