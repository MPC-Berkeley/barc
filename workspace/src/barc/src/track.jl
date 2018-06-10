#!/usr/bin/env julia

#=
	File name: track.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

using JLD
using PyPlot
using PyCall
using Distances
# pygui(true)



type Track
	ds::Float64
	width::Float64
	total_length::Float64
	thetas::Array{Float64}
	curvature

	xy_coords::Array{Float64}
	xy_outer::Array{Float64}
	xy_inner::Array{Float64}
	s_coord::Array{Float64}

	shape::AbstractString

	Track() = new()
end

function init!(track::Track)  # Standard Constructor
	println("JULIA")
	# track.shape = "oval"
	# track.shape = "track_2"
	track.shape = TRACK_NAME
	println(track.shape)
	track.ds = 1 // 10
	@assert track.ds > 0.
	track.width = TRACK_WIDTH

	track.total_length = 0.0

	track.thetas = [0.0]  # initialize theta
	track.curvature = []  # initialize curvature
	track.xy_coords = [0.0 0.0]  # initialize x and y
	track.xy_outer = [0.0 0.0]
	track.xy_inner = [0.0 0.0]

	tracks_dir = ""

	try 
		tracks_dir = readdir(TRACK_DIR)
	end

	if contains(tracks_dir, track.shape * ".jld")
		println("Loading track $(track.shape)")
		load_track!(track)
	else
		println("Creating track $(track.shape)")
		create_track!(track)
		# save_track(track)
	end
	println("total length: ", track.total_length)
end

function create_track!(track::Track)
	if track.shape == "oval"
		oval_track!(track)
	elseif track.shape == "track_2"
		track_2!(track)
	elseif track.shape == "track_3"
		track_3!(track)
	elseif track.shape == "l_shape"
		l_shape!(track)
	else
		error("Track $(track.shape) is not available.")
	end

	track.xy_coords = zeros(length(track.thetas), 2)
	track.xy_outer = zeros(length(track.thetas), 2)
	track.xy_inner = zeros(length(track.thetas), 2)
	track.xy_outer[1, 2] = track.width / 2
	track.xy_inner[1, 2] = - track.width / 2

	track.s_coord = track.ds * ones(length(track.thetas))
	track.s_coord[1] = 0.0
	track.s_coord = cumsum(track.s_coord)

	for i = 2 : length(track.thetas)
		theta = track.thetas[i]
		delta_theta = track.thetas[i] - track.thetas[i - 1]

		# TODO: Consider the transition from one radius to another 
		# radius
		if abs(delta_theta) > 1e-5 && get_curvature_check(track, track.s_coord[i]) > 1e-5
			radius = 1 / get_curvature_check(track, track.s_coord[i])

			# TODO: Should be absolute radius??
			chord = 2 * radius * sin(delta_theta / 2)

			# track.xy_coords[i, :] = chord * [cos(theta) sin(theta)]

			initial_point = track.xy_coords[1, :]
            s_direction = [initial_point[1] + chord; initial_point[2]]
            s_rotated = rotate_around_z(s_direction, track.thetas[i - 1] + delta_theta / 2)
            track.xy_coords[i, :] = s_rotated' + track.xy_coords[i - 1, :]
		else
			# in straight segments the track.ds is not curved and can 
			# therefore be directly used
			track.xy_coords[i, :] = track.xy_coords[i - 1, :] + track.ds * [cos(theta) sin(theta)]
		end
		track.xy_outer[i, :] = track.width / 2 * [cos(theta + pi / 2) 
							   sin(theta + pi / 2)]
		track.xy_inner[i, :] = track.width / 2 * [cos(theta - pi / 2) 
		  					   sin(theta - pi / 2)]
	end

	if track.shape == "l_shape"
		track.xy_coords[140 : 160, 2] -= cumsum(0.0505319492481863 / 21 * ones(21))
		track.xy_coords[161 : end, 2] -= 0.0505319492481863
	end

	# track.xy_coords = cumsum(track.xy_coords)
	track.xy_outer += track.xy_coords
	track.xy_inner += track.xy_coords

	#=
	println(track.xy_coords[1, :])
	println(track.xy_coords[end, :])
	plot(track.xy_coords[:, 1], track.xy_coords[:, 2], "ko")
	axis("equal")
	plt[:show]()
	plt[:pause](30.0)
	=#

	# Make sure that the start and end point of the track align
	if sum(abs2(track.xy_coords[1, :] - track.xy_coords[end, :])) > 1e-2
		println(track.xy_coords[1, :])
		println(track.xy_coords[end, :])
		error("Track starting and end points don't align")
	end
	# @assert sum(abs2(track.xy_coords[1, :] - track.xy_coords[end, :])) < 1e-3

	# track.total_length = round(Int64, length(track.thetas) * track.ds)
	# track.total_length = (length(track.thetas) - 1) * track.ds
end

function oval_track!(track::Track)
	add_segment!(track, 1.0, 0.0)
	add_segment!(track, 4.5, 1.0 * pi)
	add_segment!(track, 2.0, 0.0)
	add_segment!(track, 4.5, 1.0 * pi)
	add_segment!(track, 1.0, 0.0)

	#=
	add_segment!(track, 1.0, 0.0)
	add_segment!(track, 4.5, - pi)
	add_segment!(track, 2.0, 0.0)
	add_segment!(track, 4.5, - pi)
	add_segment!(track, 1.0, 0.0)
	=#

	#=
	add_segment!(track, 2.0, 0.0)
	add_segment!(track, 9.0, - pi)
	add_segment!(track, 4.0, 0.0)
	add_segment!(track, 9.0, - pi)
	add_segment!(track, 2.0, 0.0)
	=#

	#=
	add_segment!(track, 6.0, 0.0)
	add_segment!(track, 14.0, - pi)
	add_segment!(track, 12.0, 0.0)
	add_segment!(track, 14.0, - pi)
	add_segment!(track, 6.0, 0.0)
	=#
end

function l_shape!(track::Track)
	#=
	add_segment!(track, 1.5, 0.0)
	add_segment!(track, 2.2, 1.0 * pi / 2)
	add_segment!(track, 0.1, 0.0)
	add_segment!(track, 2.2, 1.0 * pi / 2)
	add_segment!(track, 0.1, 0.0)
	add_segment!(track, 2.2, - 1.0 * pi / 2)
	add_segment!(track, 0.1, 0.0)
	add_segment!(track, 2.2, 1.0 * pi / 2)
	add_segment!(track, 0.1, 0.0)
	add_segment!(track, 2.2, 1.0 * pi / 2)
	add_segment!(track, 3.0, 0.0)
	add_segment!(track, 2.2, 1.0 * pi / 2)
	add_segment!(track, 1.5, 0.0)
	=#

	add_segment!(track, 1.5, 0.0)
	add_segment!(track, 2.2, pi / 2)
	add_segment!(track, 0.1, 0.0)
	add_segment!(track, 2.2, pi / 2)
	add_segment!(track, 0.1, 0.0)
	add_segment!(track, 2.2, - pi / 2)
	add_segment!(track, 0.1, 0.0)
	add_segment!(track, 2.2, pi / 2)
	add_segment!(track, 0.1, 0.0)
	add_segment!(track, 2.2, pi / 2)
	add_segment!(track, 3.0, 0.0)
	add_segment!(track, 2.2, pi / 2)
	add_segment!(track, 1.5, 0.0)
end

function track_3!(track::Track)
	denom = 3.5
	small = false
	
	if small
		add_segment!(track, 1.7, 0.0)
	    add_segment!(track, 2.0, - pi / 2)
	    add_segment!(track, 0.5, 0.0)
	    add_segment!(track, 2.0, - pi / 2)
	    add_segment!(track, 1.2, pi / (2 * denom))
	    add_segment!(track, 1.2, - pi / denom)
	    add_segment!(track, 1.2, pi / (2 * denom))
	    add_segment!(track, 2.0, - pi / 2)
	    add_segment!(track, 0.5, 0.0)
	    add_segment!(track, 2.0, - pi / 2)
	    add_segment!(track, 1.8, 0.0)
  	else
  		add_segment!(track, 1.5, 0.0)
	    add_segment!(track, 2.0, 1.0 * pi / 2)
	    add_segment!(track, 1.5, 0.0)
	    add_segment!(track, 2.0, 1.0 * pi / 2)
	    add_segment!(track, 3.0, 0.0)
	    add_segment!(track, 2.0, 1.0 * pi / 2)
	    add_segment!(track, 1.5, 0.0)
	    add_segment!(track, 2.0, 1.0 * pi / 2)
	    add_segment!(track, 1.5, 0.0)

  		#=
		add_segment!(track, 3.0, 0.0)
	    add_segment!(track, 2.0, - pi / 2)
	    add_segment!(track, 2.0, 0.0)
	    add_segment!(track, 2.0, - pi / 2)
	    add_segment!(track, 2.0, pi / (2 * denom))
	    add_segment!(track, 2.0, - pi / denom)
	    add_segment!(track, 2.0, pi / (2 * denom))
	    add_segment!(track, 2.0, - pi / 2)
	    add_segment!(track, 2.0, 0.0)
	    add_segment!(track, 2.0, - pi / 2)
	    add_segment!(track, 2.8, 0.0)
	    =#
	end
end

function add_segment!(track::Track, length::Float64, angle::Float64)
	num_pieces = round(Int64, length / track.ds)
	# radius = length / angle
	curvature = angle / length  # 1 / radius

	angle_per_segment = angle / num_pieces
	thetas_segment = angle_per_segment * ones(num_pieces)
	thetas_segment[1] += track.thetas[end]
	append!(track.thetas, cumsum(thetas_segment))
	# append!(track.curvature, curvature * ones(num_pieces))

	start_interval = track.total_length
	end_interval = start_interval + length
	kappa = [(start_interval, end_interval), curvature]
	if size(track.curvature, 1) == 0
		track.curvature = [kappa]
	else
		track.curvature = [track.curvature kappa]
	end
	track.total_length += length
end

function get_s_coord(track::Track, s)
	if length(s) == 1
		return s % track.total_length 
	elseif length(s) == 6
		s[1] = s[1] % track.total_length
		return s
	else
		error("s-coordinate is not matching any of the predefined types.")
	end
end

function get_curvature(track::Track, s::Float64)
	s = get_s_coord(track, s)

	for i = 1 : size(track.curvature, 2)
		start_interval = track.curvature[1, i][1]
		end_interval = track.curvature[1, i][2]

		if s >= start_interval && s < end_interval
			return track.curvature[2, i]
		end
		if s <= start_interval && abs(track.curvature[2, i]) > 1e-5
			return track.curvature[2, i]
		end
	end
end

function get_curvature_check(track::Track, s::Float64)
	delta = 1e-5
	s_plus = s + delta
	s_minus = s - delta

	curvature_plus = get_curvature(track, s_plus)
	curvature_minus = get_curvature(track, s_minus)

	if abs(curvature_plus) > abs(curvature_minus)
		return curvature_plus
	else
		return curvature_minus
	end
end

function get_theta(track::Track, s::Float64)
	s = get_s_coord(track, s)

	distances_to_s = colwise(Euclidean(), track.s_coord[:, 1]', [s])
	s_indeces = sortperm(distances_to_s)[1 : 2]
	delta_theta = track.thetas[s_indeces[2]] - track.thetas[s_indeces[2]]
	theta = track.thetas[s_indeces[1]] + delta_theta / track.ds

	return theta
end

function save_track(track::Track)
	filename = ascii(TRACK_DIR * track.shape * ".jld")
	jldopen(filename, "w") do file
		JLD.write(file, "total_length", track.total_length)
		JLD.write(file, "thetas", track.thetas)
		JLD.write(file, "curvature", track.curvature)
		JLD.write(file, "xy_coords", track.xy_coords)
		JLD.write(file, "xy_outer", track.xy_outer)
		JLD.write(file, "xy_inner", track.xy_inner)
		JLD.write(file, "s_coord", track.s_coord)
	end
end

function save_track(track::Track, filename::AbstractString)
	jldopen(filename, "w") do file
		JLD.write(file, "track", track)
	end
end

function load_track!(track::Track)
	Data = load(ascii(TRACK_DIR * track.shape * ".jld"))
    track.total_length = Data["total_length"]
    track.thetas = Data["thetas"]
    track.curvature = Data["curvature"]
    track.xy_coords = Data["xy_coords"]
    track.xy_outer = Data["xy_outer"]
    track.xy_inner = Data["xy_inner"]
    track.s_coord = Data["s_coord"]
end

function load_track!(track::Track, filename::AbstractString)
	Data = load(filename)
    track = Data["track"]
end
