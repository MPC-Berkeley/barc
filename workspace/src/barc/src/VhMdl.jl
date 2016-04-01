###################### Struct for vehicle parameters
# vehicle model at steady state with constant longitudinal velocity
type VhMdl
  a::Float64          # distance from CoG to front axel
  b::Float64          # distance from CoG to rear axel
  m::Float64          # mass [kg]
  I_z::Float64        # moment of inertia about z-axis [mg*m^2]
  v_x::Float64        # reference longitudinal velocity [m/s]
  dt::Float64         # model time step discretization
end

# tire model
type TMmdl
  F::Array
  R::Array
end

###################### Lateral force from pajecka tire model
function F_paj_simp(alpha, p_TM)
  (B,C,D) = p_TM
  return D*sin(C*atan(B * alpha))
end

###################### Two-state Non-Linear approximate bicycle model
# Z = [beta ; r]
# u = delta
function f_NL_2s_approx(Z, u, Vh, TM)
  # extract states / inputs
  (beta,r) = Z
  delta     = u
  (a, b, m, I_z, v_x, dt) = (Vh.a, Vh.b, Vh.m, Vh.I_z, Vh.v_x, Vh.dt)

  # compute slip angles
  alpha_F   = atan(beta + a*r/v_x) - delta
  alpha_R   = atan(beta - b*r/v_x)

  # compute lateral forces
  FyF   = F_paj_simp(alpha_F, TM)
  FyR   = F_paj_simp(alpha_R, TM)

  # update dynamics
  beta_next  = beta + dt/(m*v_x) * (FyF + FyR) - dt*r
  r_next  = r + dt/(I_z)   * (a*FyF - b*FyR)
  return [beta_next; r_next]
end

###################### Two-state LTI approximate bicycle model
# Z = [beta ; r]
# u = [FyF ; FyR]
function f_LTI_2s_approx(Z, u, Vh, TM)
  # extract states / inputs
  (beta,r)       = Z
  (FyF, FyR)  = u
  (a, b, m, I_z, v_x, dt) = (Vh.a, Vh.b, Vh.m, Vh.I_z, Vh.v_x, Vh.dt)

  # update dynamics
  beta_next  = beta + dt*(-r + (FyF   +   FyR)/(m*v_x))
  r_next  = r + dt *   (a*FyF  -  b*FyR)/I_z
  return [beta_next; r_next]
end

###################### Two-state Non-Linear approximate bicycle model
# Z = [v_y ; r]
# u = delta
function f_NL_2s_exact(Z, u, Vh, TM)
  # extract states / inputs
  (v_y, r) = Z
  delta     = u
  (a, b, m, I_z, v_x, dt) = (Vh.a, Vh.b, Vh.m, Vh.I_z, Vh.v_x, Vh.dt)

  # compute slip angles
  alpha_F   = atan((v_y + a*r)/v_x) - delta
  alpha_R   = atan((v_y - b*r)/v_x)

  # compute lateral forces
  FyF   = F_paj_simp(alpha_F, TM)
  FyR   = F_paj_simp(alpha_R, TM)

  # update dynamics
  v_y_next  = v_y + dt*(-r*v_x + (FyF*cos(delta) +   FyR)/m )
  r_next    = r   + dt/I_z   * (a*FyF*cos(delta) - b*FyR)
  return [v_y_next; r_next]
end

###################### Three-state Non-Linear approximate bicycle model
# Z = [v_x; beta ; r]
# u = [delta; F_x]
function f_NL_3s_approx(Z, u, Vh, TM)
  # extract states / inputs
  (v_x, beta, r) = Z
  (delta, FxR)      = u
  (a, b, m, I_z, _, dt) = (Vh.a, Vh.b, Vh.m, Vh.I_z, Vh.v_x, Vh.dt)

  # compute slip angles
  alpha_F   = atan(beta + a*r/v_x) - delta
  alpha_R   = atan(beta - b*r/v_x)

  # compute lateral forces
  FyF   = F_paj_simp(alpha_F, TM.F)
  FyR   = F_paj_simp(alpha_R, TM.R)

  # update dynamics
  v_x_next  = v_x  + dt*( r*v_x*beta + (FxR -   FyF*sin(delta))/m )
  beta_next = beta + dt*(-r + (FyF +   FyR)/(m*v_x) )
  r_next    = r    + dt   * (a*FyF - b*FyR)/I_z
  return [v_x_next; beta_next; r_next]
end

###################### Three-state Non-Linear exact bicycle model
# Z = [v_x; v_y ; r]
# u = [delta; F_x]
function f_NL_3s_exact(Z, u, Vh, TM)
  # extract states / inputs
  (v_x, v_y, r) = Z
  (delta, FxR)      = u
  (a, b, m, I_z, _, dt) = (Vh.a, Vh.b, Vh.m, Vh.I_z, Vh.v_x, Vh.dt)

  # compute slip angles
  alpha_F   = atan((v_y + a*r)/v_x) - delta
  alpha_R   = atan((v_y - b*r)/v_x)

  # compute lateral forces
  FyF   = F_paj_simp(alpha_F, TM.F)
  FyR   = F_paj_simp(alpha_R, TM.R)

  # update dynamics
  v_x_next  = v_x + dt*( r*v_y + (FxR -   FyF*sin(delta))/m )
  v_y_next  = v_y + dt*(-r*v_x + (FyF*cos(delta) +   FyR)/m )
  r_next    = r   +       dt * (a*FyF*cos(delta) - b*FyR)/I_z
  return [v_x_next; v_y_next; r_next]
end

function f_NL_6s_exact(Z, u, Vh, TM)
  # extract states / inputs
  (X, Y, psi, v_x, v_y, r) = Z
  (delta, FxR)      = u
  (a, b, m, I_z, _, dt) = (Vh.a, Vh.b, Vh.m, Vh.I_z, Vh.v_x, Vh.dt)

  # compute slip angles
  alpha_F   = atan((v_y + a*r)/v_x) - delta
  alpha_R   = atan((v_y - b*r)/v_x)

  # compute lateral forces
  FyF   = F_paj_simp(alpha_F, TM.F)
  FyR   = F_paj_simp(alpha_R, TM.R)

  # update dynamics
  X_next    = X   + dt*(v_x*cos(psi) - v_y*sin(psi))
  Y_next    = Y   + dt*(v_x*sin(psi) + v_y*cos(psi))
  psi_next    = psi   + dt*r
  v_x_next  = v_x + dt*( r*v_y + (FxR -   FyF*sin(delta))/m )
  v_y_next  = v_y + dt*(-r*v_x + (FyF*cos(delta) +   FyR)/m )
  r_next    = r   +       dt * (a*FyF*cos(delta) - b*FyR)/I_z
  return [X_next; Y_next; psi_next; v_x_next; v_y_next; r_next]
end
