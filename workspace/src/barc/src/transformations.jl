#!/usr/bin/env julia

#=
	File name: transformations.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

using Distances

include("track.jl")

function s_to_xy(track::Track, s_coord::Array{Float64})
	# TODO: handle the case for the first and final element
	# TODO: assuming the s state to have 4 elements
	# TODO: rename variables
	if size(s_coord)[1] == 6
		s, e_y, e_psi, psi_dot, v_x, v_y = s_coord
	elseif size(s_coord)[1] == 5
		s, e_y, e_psi, v_x, v_y = s_coord
	else 
		s, e_y, e_psi, v = s_coord
	end

	s = get_s_coord(track, s)

	# find the two xy in which interval the s_coord lies
	distances_to_track = colwise(Euclidean(), track.s_coord[:, 1]', [s])
	two_closest_indices = sortperm(distances_to_track)[1:2]
	
	if distances_to_track[findmax(two_closest_indices)[1]] < 1e-5
		xy1_index = findmax(two_closest_indices)[1]
	else
		xy1_index = findmin(two_closest_indices)[1]
	end

	s1_xy = track.xy_coords[xy1_index, :]

	theta = track.thetas[xy1_index]
    abs_delta_theta = abs(track.thetas[xy1_index + 1] - theta)
    delta_s = s - track.s_coord[xy1_index]

	if abs_delta_theta > 1e-5 && abs(get_curvature(track, s)) > 1e-5
		curvature = get_curvature(track, s)

		if curvature < 0.0
            delta_theta = - abs_delta_theta
        else
            delta_theta = abs_delta_theta
        end

        delta_theta_prime = delta_theta / track.ds * delta_s

		r = 1.0 / curvature - e_y
		chord = 2.0 * abs(r) * sin(abs(delta_theta_prime) / 2.0)
	else
		chord = delta_s
        delta_theta = abs_delta_theta
        delta_theta_prime = delta_theta / track.ds * delta_s
	end

	theta_prime = theta + delta_theta_prime / 2.0

	initial_point = track.xy_coords[1, :]
	e_y_direction = [initial_point[1]; (initial_point[2] + e_y)]
	e_y_rotated = rotate_around_z(e_y_direction, theta)
	xy_from_origin = s1_xy' + e_y_rotated

	s_direction = [(initial_point[1] + chord); initial_point[2]]
    s_rotated = rotate_around_z(s_direction, theta_prime)
	e_y_prime = xy_from_origin + s_rotated

	x = e_y_prime[1]
	y = e_y_prime[2]

	psi = e_psi + theta_prime

	if size(s_coord)[1] == 6
		return [x, y, v_x, v_y, psi, psi_dot]
	elseif size(s_coord)[1] == 5
		return [x, y, v_x, v_y, psi, 0.0]
	else 
		return [x, y, v, 0.0, psi, 0.0]
	end


end


function xy_to_s(track::Track, xy_coord::Array{Float64})
	# TODO: fix this
	# TODO: Handle case when the the xy_coord is in between the first
	# and final point of s
	# TODO: assuming the xy state to have 6 elements, handle different
	# types of inputs
	x, y, v_x, v_y, psi, psi_dot = xy_coord

	# find the two s in which interval the xy_coord lies
	distances_to_track = colwise(Euclidean(), track.xy_coords', [x; y])
	two_closest_indices = sortperm(distances_to_track)[1:2]
	if 1 in two_closest_indices
		if x >= 0.
			s1_index = 1
		else
			s1_index = size(track.xy_coords)[1] - 1
		end
	else
		s1_index = findmin(two_closest_indices)[1]
	end

	s1_s = track.s_coord[s1_index]

	if abs(track.thetas[s1_index] - track.thetas[s1_index + 1]) > 1e-5
		if abs(get_curvature(track, track.s_coord[s1_index + 1]) - get_curvature(track, track.s_coord[s1_index])) > 1e-5
			if abs(get_curvature(track, track.s_coord[s1_index + 1])) > 1e-5
				curvature = get_curvature(track, track.s_coord[s1_index + 1])
				# curvature = get_curvature(track, track.s_coord[s1_index])
			else 
				curvature = get_curvature(track, track.s_coord[s1_index])
			end
		else
			curvature = get_curvature(track, track.s_coord[s1_index])
		end

		radius = 1. / curvature
		radius_abs = abs(radius)

		# Find the center of the circle segment
		initial_point = track.xy_coords[1, :]
		e_y_direction = [initial_point[1]; (initial_point[2] + radius)]
		e_y_rotated = rotate_around_z(e_y_direction, track.thetas[s1_index])
		center_of_segment = e_y_rotated + track.xy_coords[s1_index, :]'

		# Applying the law of cosines
		dist_to_s1 = distances_to_track[s1_index]
		dist_to_center_segment = colwise(Euclidean(), center_of_segment,
										 [x; y])[1]

		nominator = radius_abs^2  + dist_to_center_segment^2 - dist_to_s1^2
		denom = 2 * radius_abs * dist_to_center_segment
		if abs(nominator - denom) < 1e-5
			abs_phi = 0.0
		else
			abs_phi = acos(nominator / denom)
		end

		if curvature < 0.0 
			phi = - abs_phi
		else
			phi = abs_phi
		end

		#=
		plot([x; center_of_segment[1]], [y; center_of_segment[2]], "r-")
		plot([x; track.xy_coords[s1_index, 1]],
			 [y; track.xy_coords[s1_index, 2]], "r-")
		plot([track.xy_coords[s1_index, 1]; center_of_segment[1]],
			 [track.xy_coords[s1_index, 2]; center_of_segment[2]], "r-")
		=#

		# determining s and e_y
		# take the radius, because s gets projected onto it
		delta_s = radius_abs * abs_phi

		# TODO: might be the other way round, but it makes sense to me
		# to define it this way
		if curvature < 0.0
			e_y = radius_abs - dist_to_center_segment
		else 
			e_y = - (radius_abs - dist_to_center_segment)
		end

		# println("Theta: ", track.thetas[s1_index])
		# println("Phi: ", phi)
		e_psi = psi - (track.thetas[s1_index] + phi)
	else
		# using the law of cosines to determine the
		# s1 has the smaller s-value, can be further away or closer
		theta = track.thetas[s1_index]
		# println("Theta: ", theta)

		dist_to_s1 = distances_to_track[s1_index]
		dist_to_s2 = distances_to_track[s1_index + 1]

		if dist_to_s1 < 1e-5
			delta_s = 0.
			abs_e_y = 0.
		elseif dist_to_s1 + dist_to_s2 <= track.ds + 1e-5
			delta_s = dist_to_s1
			abs_e_y = 0.
		else
			abs_phi = acos((dist_to_s1^2 + track.ds^2 - dist_to_s2^2) /
					   (2 * dist_to_s1 * track.ds))
			if get_curvature(track, track.s_coord[s1_index + 1]) < 0.0 
				phi = abs_phi
			else
				phi = abs_phi
			end
			delta_s = dist_to_s1 * cos(phi)
			abs_e_y = dist_to_s1 * sin(phi)
		end

		# no phi added because it is a straight segment
		e_psi = psi - theta

		initial_point = track.xy_coords[1, :]
		e_y_direction = [initial_point[1]; (initial_point[2] +
						(- track.width / 2))]
						# (track.width / 2))]
		e_y_rotated = rotate_around_z(e_y_direction, theta)
		xy_position = track.xy_coords[s1_index, :]' +
					  delta_s * [cos(theta); sin(theta)]
					  # [delta_s; 0.0]
		center_of_segment = e_y_rotated + xy_position
		distance_to_inner_bound = colwise(Euclidean(), center_of_segment,
										  [x; y])
		if distance_to_inner_bound[1] <= track.width / 2
			e_y = abs_e_y
		else
			e_y = - abs_e_y
		end
	end

	s = s1_s + delta_s

	# println("e_psi before: ", e_psi)

	#=
	if e_psi < - pi 
		e_psi = pi + 
		e_psi -= floor(e_psi / (2 * pi)) * 2 * pi  # min(ceil(e_psi / (2 * pi)), - 1.0) * 2 * pi
	elseif e_psi > pi
		e_psi -= ceil(e_psi / (2 * pi)) * 2 * pi  # max(floor(e_psi / (2 * pi)), 1.0) * 2 * pi
	end
	=#
	e_psi = atan2(sin(e_psi), cos(e_psi))
	# e_psi = (e_psi + pi) % (2 * pi) - pi

	# println("e_psi after: ", e_psi)

	# (self.psi - self.theta[self.idx_curr]+pi)%(2*pi)-pi

	return [s, - e_y, e_psi, psi_dot, v_x, v_y]
end


function rotate_around_z(xy_coords::Array{Float64}, angle::Float64)
	@assert size(xy_coords)[1] == 2
	rotation_matrix = [cos(angle) (- sin(angle)); sin(angle) cos(angle)]

	return rotation_matrix * xy_coords
end
