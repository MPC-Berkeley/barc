
# obstacle parameters

# obstacle representation for optimization problem
nOb =  3 			# number of obstacles 
vOb = [3 3 2]		# number of vertices of each obstacle, vector of dimenion nOb
vObMPC = vOb-1		# adjustment for optimizaton problem

# for optimization problem
lOb = [   [ [-20;5], [-1.3;5], [-1.3;-5]]  , 
 	  [ [1.3;-5] , [1.3;5] , [20;5] ] , 
	  [ [20;11], [-20;11]]	]		#vetices given in CLOCK-WISE direction

# for plotting
lObPlot = [   [ [-20;5], [-1.3;5], [-1.3;-5], [-20;-5], [-20;5] ]  ,
      [ [1.3;5], [20;5], [20;-5], [1.3;-5], [1.3;5] ] ,
      [ [-20;15], [20;15], [20;11], [-20,11], [-20;15] ]        ]       #vetices given in CLOCK-WISE direction
nObPlot =  3        # number of obstacles
vObPlot = [4 4 4]   # number of vertices of each obstacle, vector of dimenion nOb

# build obstacles for Hybrid A* algorithm
ox = Float64[]
oy = Float64[]

# obstacle 1
for i = -12:0.1:-1.3
    push!(ox, Float64(i))
    push!(oy, 5.0)
end
for i in -2:5
    push!(ox, -1.3)
    push!(oy, Float64(i))
end


# obstacle 2
for i in -2:5
    push!(ox, 1.3)
    push!(oy, Float64(i))
end
for i = 1.3:0.1:12
    push!(ox, Float64(i))
    push!(oy, 5.0)
end


# obstacle 3
for i = -12:12
    push!(ox, Float64(i))
    push!(oy, 11.0)
end

#           [x_lower, x_upper, -y_lower,   y_upper  ]
XYbounds =  [ -15   , 15      ,  1      ,  10       ]