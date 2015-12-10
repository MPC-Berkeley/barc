# stateEstimation
from numpy import array, dot, cos, sin

def estimateAngularAcceleration(imu_data, wz_k, t_k, dt):
    # compute dot_w_z
    (_, _, _, wz_kp1)    = imu_data.getFilteredSignal()
    if t_k > 0:
        dwz = (wz_kp1 - wz_k)/dt
    else:
        dwz = 0
    return dwz

# estimate velocity in the vehicle frame
def estimateVelocity(imu_data, v_BF, vx_enc, dwz, d_F, mdl, dt):
    
    # unpack filtered imu data
    # note: km1 = k minus 1, kp1 = k plus one
    (_,_,a,_) = mdl
    (_, ay_IMU, _, w_z)    = imu_data.getFilteredSignal()
    (_ , vy_CoG_km1)         = v_BF.getRawSignal()
    
    # compute v_x
    # 1. project encoder reading for front wheel to body frame x-axis (via cosine term)
    vx_CoG_k = 2*vx_enc*cos(d_F)
    
    # estimate v_x and v_y at CoG_
    if vx_CoG_k != 0:
        ay_CoG = ay_IMU - a*dwz
        vy_CoG_k = vy_CoG_km1 +  (ay_CoG - w_z*vx_CoG_k)*dt 
    else:
        vy_CoG_k = 0    
    
    v_BF.update(list([vx_CoG_k, vy_CoG_k])) 


       
# estimate position in the global frame
def estimatePosition( v_BF, X_GF, psi, vx_enc, dt):
    """
    function    : estimate_position
    
    input       : current velocity estimates in the body frame, and 
                  position estimates in the global frame
    output      : current state estimate for (v_x, v_y) in local frame and 
                  (X,Y,v_X, v_Y) in global frame
    
    assumptions: a_z = v_z = w_x = w_y = 0
    """
    
    #### unpack signal data
    (v_x , v_y)             = v_BF.getRawSignal()    
    X_t                     = X_GF.getFilteredSignal()
    
    # compute the position    
    A = array([[1,      0,      dt,     0],
               [0,      1,      0,      dt],	
               [0,      0,      0,      0],
               [0,      0,      0,      0],])
    B = array([[0,0],
               [0,0],
               [1,0],
               [0,1]])
    R = array([[cos(psi),   sin(psi)],
               [-sin(psi),  cos(psi)]])
    u = dot(R, array([v_x,v_y]))
    X_next = dot(A, X_t) + dot(B,u)
    X_GF.update(list(X_next))