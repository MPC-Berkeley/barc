using NPZ
using JuMP
using Ipopt

type CornerStiffnessIdentification
    mdl::JuMP.Model

    v_x::Array{JuMP.NonlinearParameter,1}
    d_f::Array{JuMP.NonlinearParameter, 1}
    psi_dot::Array{JuMP.NonlinearParameter,1}

    c_f::JuMP.Variable
    c_r::JuMP.Variable
    beta::Array{JuMP.Variable, 1}

    alpha_f::Array{JuMP.NonlinearExpression, 1}
    alpha_r::Array{JuMP.NonlinearExpression, 1}
    F_yf::Array{JuMP.NonlinearExpression, 1}
    F_yr::Array{JuMP.NonlinearExpression, 1}

    cost::JuMP.NonlinearExpression

    function CornerStiffnessIdentification(num_experiments::Int64)
        println("Starting identification of corner stiffness")
        m = new()
        
        l_f = 0.125
        l_r = 0.125
        mass = 2.0  # old Barc
        # mass = 1.75  # new Barc
        I_z = 0.24

        # Create Model
        mdl = Model(solver = IpoptSolver(print_level=0, linear_solver="ma27"))

        # Create variables (these are going to be optimized)
        @variable(mdl, c_f >= 0, start = 0) 
        @variable(mdl, c_r >= 0, start = 0)
        @variable(mdl, beta[1 : num_experiments], start = 0)

        # Set bounds
        beta_lb = ones(num_experiments) * (- 1.0)   # lower bound on beta
        beta_ub = ones(num_experiments) * 1.0 # upper bound on beta

        for i = 1 : num_experiments
            setlowerbound(beta[i], beta_lb[i])
            setupperbound(beta[i], beta_ub[i])
        end

        @NLparameter(mdl, v_x[i = 1 : num_experiments] == 0)
        @NLparameter(mdl, d_f[i = 1 : num_experiments] == 0)
        @NLparameter(mdl, psi_dot[i = 1 : num_experiments] == 0)

        @NLexpression(mdl, alpha_f[i = 1 : num_experiments], atan(beta[i] + l_f * psi_dot[i] / v_x[i]) - d_f[i])
        @NLexpression(mdl, alpha_r[i = 1 : num_experiments], atan(beta[i] - l_r * psi_dot[i] / v_x[i]))
        @NLexpression(mdl, F_yf[i = 1 : num_experiments], - alpha_f[i] * c_f)
        @NLexpression(mdl, F_yr[i = 1 : num_experiments], - alpha_r[i] * c_r)

        @NLexpression(mdl, cost_translation, sum{(1 / (mass * v_x[i]) * (F_yf[i] + F_yr[i]) + psi_dot[i])^2, i = 1 : num_experiments})
        @NLexpression(mdl, cost_rotation, sum{(1 / I_z * (l_f * F_yf[i] - l_r * F_yr[i]))^2, i = 1 : num_experiments})

        @NLobjective(mdl, Min, cost_translation + cost_rotation)
        
        m.mdl = mdl
        m.c_f = c_f
        m.c_r = c_r
        m.beta = beta
        m.v_x = v_x
        m.d_f = d_f
        m.psi_dot = psi_dot
        m.alpha_f = alpha_f
        m.alpha_r = alpha_r
        m.F_yf = F_yf
        m.F_yr = F_yr
        return m
    end
end

function identifiy_corner_stiffness(mdl::CornerStiffnessIdentification, v_x::Array{Float64}, 
                                    psi_dot::Array{Float64}, d_f::Array{Float64})
    mass = 1.75
    sol_status::Symbol
    optimal_cf::Float64
    optimal_cr::Float64

    setvalue(mdl.v_x, v_x)
    setvalue(mdl.d_f, d_f)
    setvalue(mdl.psi_dot, psi_dot) 

    # Solve Problem and return solution
    sol_status = solve(mdl.mdl)
    optimal_cf = getvalue(mdl.c_f)
    optimal_cr = getvalue(mdl.c_r)

    println("max alpha_f: ", findmax(getvalue(mdl.alpha_f))[1])
    println("max alpha_r: ", findmax(getvalue(mdl.alpha_r))[1])
    # beta_order = sortperm(getvalue(mdl.beta))
    # println("beta: ", getvalue(mdl.beta)[beta_order])
    println("beta: ", getvalue(mdl.beta))
    beta = getvalue(mdl.beta)


    println("Solved, status = $(sol_status)")
    println("Optimal C_f: ", optimal_cf)
    println("Optimal C_r: ", optimal_cr)
    
    indices = abs(beta) .< 0.7
    println(beta[indices])
    alpha_f = getvalue(mdl.alpha_f)[indices]
    alpha_r = getvalue(mdl.alpha_r)[indices]
    F_yf = getvalue(mdl.F_yf)[indices]
    F_yr = getvalue(mdl.F_yr)[indices]
    println(getvalue(mdl.alpha_f)[indices])
    println(getvalue(mdl.alpha_r)[indices])

    println("Max Force: ", mass * 9.81 / 2)
    println("Fyf: ", findmax(F_yf)[1])
    println("Fyr: ", findmax(F_yr)[1])

    return indices
end


dir = homedir() * "/barc_debugging/corner_stiffness/"
folders = readdir(dir)
num_experiments = size(folders, 1)

average_vx = zeros(num_experiments)
average_psidot = zeros(num_experiments)
average_df = zeros(num_experiments)

for i = 1 : num_experiments
    # old Barc: 
    data_vx = 0.5 * (npzread(dir * folders[i] * "/estimator_enc.npz")["v_rr_his"] + npzread(dir * folders[i] * "/estimator_enc.npz")["v_rl_his"])
    # new Barc: 
    # data_vx = npzread(dir * folders[i] * "/estimator_enc.npz")["v_rr_his"]
    data_psidot = npzread(dir * folders[i] * "/estimator_imu.npz")["psiDot_his"]

    average_vx[i] = mean(data_vx[100 : end - 50])
    average_psidot[i] = mean(data_psidot[100 : end - 50])
    average_df[i] = float(split(folders[i], "_")[1])

    println(size(data_vx))
end

mdl = CornerStiffnessIdentification(num_experiments)
# mdl.CornerStiffnessIdentification(num_experiments)

indices = identifiy_corner_stiffness(mdl, average_vx, average_psidot, average_df)

num_experiments = size(average_vx[indices], 1)
mdl2 = CornerStiffnessIdentification(num_experiments)

println(average_vx[indices])
println(average_psidot[indices])
println(average_df[indices])
identifiy_corner_stiffness(mdl2, average_vx[indices], average_psidot[indices], average_df[indices])
