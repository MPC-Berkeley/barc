#!/usr/bin/env julia

using RobotOS
using JuMP, Ipopt
using JLD
@rosimport barc.msg: ECU, Z_KinBkMdl, mpcSol
@rosimport std_msgs.msg: Float32
rostypegen()
using barc.msg
using std_msgs.msg
include("modules/mpcPathFollowing.jl")

function euclideanDistance(s1::Array{Float64},s2::Array{Float64})
	dx = s1[1] - s2[1]
	dy = s1[2] - s2[2]
	return sqrt( dx^2 + dy^2 )
end

function updateState(msg::Z_KinBkMdl, 
					sCurr::Array{Float64}, 
					sPrev::Array{Float64}, 
					dist::Array{Float64}, 
					sRef::Array{Float64,2},
					uRef::Array{Float64,2},
					sp10Ref::Array{Float64,2}, 
					up10Ref::Array{Float64,2},
					dp10Ref::Array{Float64},
					N::Int64)
    # get state
    sPrev[:] = copy(sCurr)
    sCurr[:] = [msg.x , msg.y , msg.psi , msg.v]

    # compute distance travelled in previous time step
    delD  	   = euclideanDistance(sCurr, sPrev)
    dist[:]    = dist[1] + delD
    idx 	   = indmin( (dp10Ref - dist[1]).^2 )

	sRef[:,:] = copy( sp10Ref[:,idx:idx+N] )
	uRef[:,:] = copy( up10Ref[:,idx:idx+N-1] )
end

function main()
    # initiate node, set up publisher / subscriber topics
    println("loading parameters ...")
    s0 			= get_param("/initial_state")
    sF 			= get_param("/final_state")
    L 			= get_param("/vehicle_length")
    Ts 			= get_param("/simulation_time_step")
    N 			= get_param("/horizon_preview")
    Q 			= get_param("/state_penalty_matrix")
    R 			= get_param("/input_penalty_matrix")
    stateDim 	= size(s0)[1]
    inputDim 	= 2
    Q 			= reshape(Q,(stateDim, stateDim))
    R 			= reshape(R,(inputDim, inputDim))
	mdlParams   = MdlParams(L,Ts)
	mpcParams   = MpcParams(N,Q,R)
	sol         = MpcSol()

	println("running first optimization with a horizon preview of N = ", N, " steps ...")
	mdl    		= MpcModel(mpcParams,mdlParams)

	println("loading pre-computed solution ...")
    home        = ENV["HOME"]
	filePath 	= string(home,"/barc/workspace/src/barc/src/control/parking/data")
	data 		= load("$filePath/parkPath.jld")
	sp10 		= data["sp10"]
	up10 		= data["up10"]
	scaleTime10 = data["scaleTime10"]
	sp10Ref 	= data["sp10Ref"]
	up10Ref 	= data["up10Ref"]
	dp10Ref 	= data["dp10Ref"]

	sp10Rest 	= sp10[:,end]*ones(1,N+1)
	up10Rest 	= [0,0]*ones(1,N)
	sp10Ref 	= hcat(sp10Ref, sp10Rest)
	up10Ref 	= hcat(up10Ref, up10Rest)
	sCurr 		= copy(s0)
	sPrev 		= copy(s0)
	sRef 		= copy(sp10Ref[:,1:N+1] )
	uRef 		= copy(up10Ref[:,1:N] )
	dist  		= [0.0]
	solveMPC 	= true

	println("initilizing node ...")
    init_node("controller")
    p1 = Publisher("controller", ECU, queue_size=1)
    p2 = Publisher("mpcSolution", mpcSol, queue_size=1)
    s1 = Subscriber{Z_KinBkMdl}("plant", updateState, (sCurr,sPrev,dist,sRef,uRef,sp10Ref,up10Ref,dp10Ref,N), queue_size=1)
    loop_rate = Rate( Int(1.0/Ts) )

	println("running mpc ...")
    while ! is_shutdown()

    	if euclideanDistance(sCurr, sF) > 0.1 && solveMPC 
    		SolveMpcProblem(mdl, sol, sCurr, sRef, uRef)
    		publish(p1, ECU( sol.acc, sol.df) )
    		publish(p2, mpcSol( sol.zOL[1,:], sol.zOL[2,:], sol.zOL[3,:] , sol.zOL[4,:], sol.uOL[1,:], sol.uOL[2,:]) )
    		println(sol.solverStatus)
    	else
    		solveMPC = false
    		cmd = ECU(-sCurr[4], 0)
    		publish(p1, cmd )
    	end

        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
