# Define Variables
oldTraj         = OldTrajectory()
lapStatus       = LapStatus(1,1)
mpcCoeff        = MpcCoeff()
posInfo         = PosInfo()
mpcParams       = MpcParams()
stateIn         = zeros(7,1)        # stateIn: xdot, ydot, psidot, epsi, y, s, a_f
inputIn         = zeros(2,1)
mpcSol          = MpcSol()
trackCoeff      = TrackCoeff()      # info about track (at current position, approximated)
modelParams     = ModelParams()

# ===============================
# Initialize Parameters
# ===============================
buffersize                  = 700
oldTraj.oldTraj             = zeros(4,buffersize,2)
oldTraj.oldInput            = zeros(2,buffersize,2)

posInfo.s_start             = 0
posInfo.s_target            = 2

mpcParams.N                 = 5
mpcParams.nz                = 4
mpcParams.Q                 = [0.0 1.0 1.0 1.0]     # put weights on ey, epsi and v
mpcParams.vPathFollowing    = 0.2

mpcCoeff.coeffCost          = 0
mpcCoeff.coeffConst         = 0
mpcCoeff.order              = 5
mpcCoeff.pLength            = 4*mpcParams.N        # small values here may lead to numerical problems since the functions are only approximated in a short horizon

trackCoeff.coeffCurvature   = [0.0,0.0,0.0,0.0,0.0]         # polynomial coefficients for curvature approximation (zeros for straight line)
trackCoeff.nPolyCurvature   = 4                   # 4th order polynomial for curvature approximation
trackCoeff.width            = 0.4                 # width of the track (0.5m)

modelParams.u_lb            = [-2.0 -pi/6]' * ones(1,mpcParams.N)                    # lower bounds on steering
modelParams.u_ub            = [2.0  pi/6]' * ones(1,mpcParams.N)                    # upper bounds
modelParams.z_lb            = [-Inf -trackCoeff.width/2 -Inf -Inf]' * ones(1,mpcParams.N+1)                    # lower bounds on states
modelParams.z_ub            = [Inf   trackCoeff.width/2  Inf  Inf]' * ones(1,mpcParams.N+1)                    # upper bounds
modelParams.c0              = [0.5431, 1.2767, 2.1516, -2.4169]         # BARC-specific parameters (measured)
modelParams.l_A             = 0.25
modelParams.l_B             = 0.25
modelParams.dt              = 0.1

z_Init                      = zeros(4,1)

# =================================
# Create Model
# =================================
println("Building model...")
dt              = modelParams.dt
L_a             = modelParams.l_A
L_b             = modelParams.l_B
N               = mpcParams.N
c0              = modelParams.c0

mdl             = Model(solver = IpoptSolver(print_level=0,max_cpu_time=0.01))#,linear_solver="ma57",print_user_options="yes"))

#@variable( mdl, modelParams.z_ub[i,j] >= z_Ol[i=1:4,j=1:(N+1)] >= modelParams.z_lb[i,j])      # z = s, ey, epsi, v
@variable( mdl, z_Ol[1:4,1:(N+1)])      # z = s, ey, epsi, v
#@variable( mdl, modelParams.u_ub[i,j] >= u_Ol[i=1:2,j=1:N] >= modelParams.u_lb[i,j])      # u = a, d_f
@variable( mdl, u_Ol[1:2,1:N])

for i=1:2       # I don't know why but somehow the short method returns errors sometimes
    for j=1:N
        setlowerbound(u_Ol[i,j], modelParams.u_lb[i,j])
        setupperbound(u_Ol[i,j], modelParams.u_ub[i,j])
    end
end
#=
for i=1:4
    for j=1:N+1
        setlowerbound(z_Ol[i,j], modelParams.z_lb[i,j])
        setupperbound(z_Ol[i,j], modelParams.z_ub[i,j])
    end
end
=#

#@constraint(mdl, [i=1:2,j=1:N], u_Ol[i,j] <= modelParams.u_ub[i,j])
@variable( mdl, 1 >= ParInt >= 0 )

@NLparameter(mdl, z0[i=1:4] == z_Init[i])
@NLconstraint(mdl, [i=1:4], z_Ol[i,1] == z0[i])

@NLparameter(mdl, coeff[i=1:length(trackCoeff.coeffCurvature)] == trackCoeff.coeffCurvature[i]);

@NLexpression(mdl, c[i = 1:N],    coeff[1]*z_Ol[1,i]^4+coeff[2]*z_Ol[1,i]^3+coeff[3]*z_Ol[1,i]^2+coeff[4]*z_Ol[1,i]+coeff[5])
#@NLexpression(mdl, c[i = 1:N],    sum{coeff[j]*z_Ol[1,i]^(5-j),j=1:5})         # gives segmentation fault (probably doesn't like x^0 = 1)
@NLexpression(mdl, bta[i = 1:N],  atan( L_a / (L_a + L_b) * tan( c0[3]*u_Ol[2,i] + c0[4]*abs(u_Ol[2,i])*u_Ol[2,i]) ) )
@NLexpression(mdl, dsdt[i = 1:N], z_Ol[4,i]*cos(z_Ol[3,i]+bta[i])/(1-z_Ol[2,i]*c[i]))

# System dynamics
for i=1:N
    @NLconstraint(mdl, z_Ol[1,i+1]  == z_Ol[1,i] + dt*dsdt[i]  )                                             # s
    @NLconstraint(mdl, z_Ol[2,i+1]  == z_Ol[2,i] + dt*z_Ol[4,i]*sin(z_Ol[3,i]+bta[i])  )                     # ey
    @NLconstraint(mdl, z_Ol[3,i+1]  == z_Ol[3,i] + dt*(z_Ol[4,i]/L_a*sin(bta[i])-dsdt[i]*c[i])  )            # epsi
    @NLconstraint(mdl, z_Ol[4,i+1]  == z_Ol[4,i] + dt*(c0[1]*u_Ol[1,i] - c0[2]*abs(z_Ol[4,i]) * z_Ol[4,i]))  # v
end