#=
simple_simulator:
- Julia version: 
- Author: lukas
- Date: 2018-04-03
=#


function simulate_xy(agent::Agent, inputs::Array{Float64})
	dt = agent.dt
	delta_dt = dt / 10
	delta_dt = dt

	horizon = size(inputs)[1]
	z_next = zeros(horizon + 1, size(agent.states_xy)[2])

	prediction = 1
	iteration = agent.current_iteration

	for i = iteration : iteration + horizon
		if prediction == 1
			z_next[prediction, :] = get_state_xy(agent, i)
			prediction += 1
			continue
		else
			z_next[prediction, :] = z_next[prediction - 1, :]
		end

		# extra for loop to simulate more accuratley
		# this results in the actual specified amount of iterations (10)
		for t = delta_dt : delta_dt : dt
			if agent.is_dynamic
				z_next[prediction, :] = simulate_xy_kin(agent,
														z_next[prediction, :],
														inputs[prediction - 1, :],
														delta_dt)
			else
				z_next[prediction, :] = simulate_xy_kin(agent,
														z_next[prediction, :],
														inputs[prediction - 1, :],
														delta_dt)
			end
		end
		prediction += 1
	end

	return z_next
end

function simulate_xy_dyn(agent::Agent, state::Array{Float64},
						 input::Array{Float64}, dt::Float64)
	# dynamic bicycle model, therefore taking into account forces

	x, y, v_x, v_y, psi, psi_dot = state

	l_front = agent.l_front
	l_rear = agent.l_rear
	mass = agent.mass
	mu = agent.mu
	max_slip_angle = agent.max_slip_angle
	g = agent.g
	C = agent.C
	B = agent.B
	I_z = agent.I_z

	F_xr = mass * input[1]
	F_max = mu * mass * g / 2.0

	if F_xr > F_max
		F_xr = F_max
	elseif F_xr < - F_max
		F_xr = - F_max
	end

	# Determine slip angles (alpha)
	if v_x < 0.1
		slip_f = 0.0
		slip_r = 0.0
	else
		slip_f = atan((v_y + l_front * psi_dot) / v_x) - input[2]
		slip_r = atan((v_y - l_rear * psi_dot) / v_x)
	end

	# TODO: edit warning
	if max(abs(slip_f), abs(slip_r)) > max_slip_angle / 180 * pi
		warn("Large slip angles")
	end

	F_yf = - F_max * sin(C * atan(B * slip_f))
	F_yr = - F_max * sin(C * atan(B * slip_r))

	if F_yr > sqrt(F_max^2 - F_xr^2)
		Fyr = sqrt(F_max^2 - F_xr^2)
	elseif F_yr < - sqrt(F_max^2 - F_xr^2)
		F_yr = - sqrt(F_max^2 - F_xr^2)
	end

	x += dt * (v_x * cos(psi) - v_y * sin(psi))
	y += dt * (v_x * sin(psi) + v_y * cos(psi))
	v_x_new = v_x + dt * (psi_dot * v_y + 1 / mass * (F_xr - F_yf *
			  sin(input[2])))
	v_y += dt * (- psi_dot * v_x + 1 / mass * (F_yf * cos(input[2]) + F_yr))
	psi += dt * psi_dot
	psi_dot += dt * (1 / I_z * (l_front * F_yf * cos(input[2]) - l_rear *
			   F_yr))

	return [x y v_x_new v_y psi psi_dot]
end

function simulate_xy_kin(agent::Agent, state::Array{Float64},
						 input::Array{Float64}, dt::Float64)
	# kinematic bicycle model

	x, y, v_x, v_y, psi, psi_dot = state

	l_front = agent.l_front
	l_rear = agent.l_rear

	beta = atan(l_rear / (l_front + l_rear) * tan(input[2]))

	x += dt * (v_x * cos(psi + beta))
	y += dt * (v_x * sin(psi + beta))
	v_x_new = v_x + dt * input[1]
	v_y = 0
	psi += dt * (v_x / l_rear * sin(beta))
	psi_dot = 0

	return [x y v_x_new v_y psi psi_dot]
end

function simulate_s_kin(agent::Agent, track::Track, inputs::Array{Float64})
	# The simulation and the optimization result in different predictions if
	# the extra for loop has more than one step. The simulation's result is
	# therefore more accurate.

	dt = agent.dt
	delta_dt = dt / 10
	delta_dt = dt

	l_front = agent.l_front
	l_rear = agent.l_rear

	horizon = size(inputs)[1]
	z_next = zeros(horizon + 1, size(agent.states_s)[2])

	prediction = 1
	iteration = agent.current_iteration

	# println("sim current state: ", get_state_s(agent, iteration))

	for i = iteration : iteration + horizon
		if prediction == 1
			z_next[prediction, :] = get_state_s(agent, i)
			prediction += 1
			continue
		else
			z_next[prediction, :] = z_next[prediction - 1, :]
		end

		input = inputs[prediction - 1, :]
		# extra for loop to simulate more accuratley
		counter = delta_dt
		for t = delta_dt : delta_dt : dt
			s, e_y, e_psi, v = z_next[prediction, :]

			beta = atan(l_rear / (l_front + l_rear) * tan(input[2]))
			curvature = get_curvature(track, s)
			s_dot = v * cos(e_psi + beta) / (1 - e_y * curvature)

			s += delta_dt * s_dot
			e_y += delta_dt * v * sin(e_psi + beta)
			e_psi += delta_dt * (v / l_rear * sin(beta) - s_dot * curvature)
			v += delta_dt * input[1]

			z_next[prediction, :] = [s e_y e_psi v]
			counter += delta_dt
		end
		prediction += 1
	end

	return z_next
end

function simulate_s_dyn(agent::Agent, track::Track, inputs::Array{Float64})
	# The simulation and the optimization result in different predictions if
	# the extra for loop has more than one step. The simulation's result is
	# therefore more accurate.

	dt = agent.dt
	delta_dt = dt / 10
	# delta_dt = dt

	l_front = agent.l_front
	l_rear = agent.l_rear
	mass = agent.mass
	mu = agent.mu
	max_slip_angle = agent.max_slip_angle
	g = agent.g
	I_z = agent.I_z

	horizon = size(inputs)[1]
	z_next = zeros(horizon + 1, size(agent.states_true_s)[2])

	prediction = 1
	iteration = agent.current_iteration

	# println("sim current state: ", get_state_s(agent, iteration))

	for i = iteration : iteration + horizon
		if prediction == 1
			z_next[prediction, :] = get_state_true_s(agent, i)
			prediction += 1
			continue
		else
			z_next[prediction, :] = z_next[prediction - 1, :]
		end

		input = inputs[prediction - 1, :]

		# extra for loop to simulate more accuratley
		counter = delta_dt
		for t = delta_dt : delta_dt : dt
			input[1] -= input[1] * delta_dt / dt
			input[2] -= input[2] * delta_dt / dt

			s, e_y, e_psi, psi_dot, v_x, v_y = z_next[prediction, :]

			v = sqrt(v_x^2 + v_y^2)
			beta = atan(l_rear / (l_front + l_rear) * tan(input[2]))

			a_front = atan((v_y + l_front * psi_dot) / abs(v_x)) - input[2]
			a_rear = atan((v_y - l_rear * psi_dot) / abs(v_x))

			# a_front = - input[2]
			# a_rear = 0.

			F_front = - 0.5 * mass * g * mu * pacejka(agent, a_front)
			F_rear = - 0.5 * mass * g * mu * pacejka(agent, a_rear)

			curvature = get_curvature(track, s)
			s_dot = (v_x * cos(e_psi) - v_y * sin(e_psi)) / (1 - e_y * curvature)

			s += delta_dt * s_dot
			e_y += delta_dt * (v_x * sin(e_psi) + v_y * cos(e_psi))
			e_psi += delta_dt * (psi_dot - curvature * s_dot)
			v_x_new = v_x + delta_dt * (input[1] + psi_dot * v_y)
			v_y += delta_dt * (2 / mass * (F_front *  cos(input[2]) + F_rear) - psi_dot * v_x)
			psi_dot += delta_dt * 2 / I_z * (l_front * F_front - l_rear * F_rear)

			z_next[prediction, :] = [s e_y e_psi psi_dot v_x_new v_y]
			# counter += delta_dt
			# println(counter)
		end
		prediction += 1
	end
	return z_next
end

function pacejka(agent::Agent, alpha::Float64)
	C = agent.C
	B = agent.B

	return sin(C * atan(B * alpha))
end
