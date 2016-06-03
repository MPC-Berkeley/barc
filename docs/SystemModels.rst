******************
Coordinate System
******************

For all vehicle models, we set a body frame coordinate system in which the positive x axis points to the front of the vehicle, the positive y axis points to the left of the vehicle, and the positive z axis points upward, as shown in the image below.

.. image:: https://github.com/BARCproject/barc/raw/master/docs/imgs/coordinate_system.PNG
   :height: 250px
   :width: 400 px
   :align: center

***************
Kinematic model
***************

Kinematic models succinctly capture the motion of the vehicle with the state vector
:math:`z=[x,y,\psi,v]^\top`, where
:math:`(x,y)` describe the position of the vehicle in the global frame,
:math:`\psi` describes the heading angle of the vehicle, and
:math:`v` is the velocity of the vehicle.
The input vector is :math:`u=[a,\delta_f]^\top`, where
:math:`a` is the acceleration, and
:math:`\delta_f` is the front steering angle we assume the r, the rear steering
angle, is zero.

.. math::
   \dot{x}    &=  v \, \cos(\psi + \beta) \\
   \dot{y}    &=  v \, \sin(\psi + \beta) \\
   \dot{\psi} &=  \frac{v \, \cos(\beta)}{l_f + l_r} \, \tan(\delta_f) \\
   \dot{v}    &=  a \\
   \beta      &=  \tan^{-1} (\frac{l_r}{l_f+l_r} \tan(\delta_f))

where :math:`l_f, l_r` are the distances from the center of mass to the front axel and rear axel,
respectively.
Note that the expression for the yaw rate may vary in the literature

.. math::
   \dot{\psi} &=  \frac{v \, \cos(\beta)}{l_f + l_r} \, \tan(\delta_f) = \frac{v}{l_r} \sin(\beta) \\

The descriptor kinematic implies that this model does not consider the mass of the vehicle.
It simply considers the motion that follows from the velocity and steering angle using geometry.
This model captures the trajectory of the vehicle well at low speeds, in which
:math:`\beta`
is small, meaning only a few degrees.

Below is a code segment of how a discretized version of the kinematic model would be written in python.
This code is found in the file system_models.py inside the barc ROS package

.. code-block:: python

  def f_KinBkMdl(z,u,vhMdl, dt):
    # get states / inputs
    x       = z[0]
    y       = z[1]
    psi     = z[2]
    v       = z[3]
    d_f     = u[0]
    a       = u[1]

    # extract parameters
    (L_a, L_b)             = vhMdl

    # compute slip angle
    bta         = arctan( L_a / (L_a + L_b) * tan(d_f) )

    # compute next state
    x_next      = x + dt*( v*cos(psi + bta) )
    y_next      = y + dt*( v*sin(psi + bta) )
    psi_next    = psi + dt*v/L_b*sin(bta)
    v_next      = v + dt*a

    return array([x_next, y_next, psi_next, v_next])

For more information, refer to the textbook `Vehicle Dynamics and Control` [Spring, 2011] Chapter 2
by R.Rajamani


***************
Kinetic model
***************

Kinetic models take into account the forces acting on the vehicle. For analysis and
design purposes, engineers commonly use either the bicycle model or the full vehicle model
depending on model fidelity requirements. These two models describe the same dynamics,
except that the bicycle model simplifies the physics of the system by lumping the front two
tires and the rear two rears.

The dynamics are obtained from Newtonian mechanics, using principles from the balance
of linear and angular momentum. For a vehicle with rear wheel drive, the equations of motion
are as shown below.

.. math::
   \dot{x}    &=  v_x \, \cos \psi - v_y \, \sin \psi \\
   \dot{y}    &=  v_x \, \sin \psi + v_y \, \cos \psi \\
   \dot{y}    &=  r \\
   \dot{v}_x  &=  v_y \, r + \frac{1}{m} (F_{xR} - F_{yF} \, \sin \delta) \\
   \dot{v}_y  &= -v_x \, r + \frac{1}{m} (F_{yF} \, \cos \delta + F_{yR} ) \\
   \dot{r}    &=  \frac{1}{I_z} \, (l_f \, F_{yR} \, \cos \delta - l_f \, F_{yR} )
