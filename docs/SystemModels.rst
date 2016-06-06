******************
Coordinate System
******************

For all vehicle models, we set a body frame coordinate system in which the positive x axis points to the front of the vehicle, the positive y axis points to the left of the vehicle, and the positive z axis points upward, as shown in the image below.

.. image:: https://github.com/BARCproject/barc/raw/master/docs/imgs/coordinate_system.PNG
   :height: 250 px
   :width: 400 px
   :align: center

***************
Nomenclature
***************
.. image:: https://github.com/BARCproject/barc/raw/master/docs/imgs/nomenclature.PNG
   :height: 400 px
   :width: 475 px
   :align: center

***************
Kinematic model
***************

Kinematic models succinctly capture the motion of the vehicle from the velocity and steering angle using geometry.

.. image:: https://github.com/BARCproject/barc/raw/master/docs/imgs/kinBkMdl_eqns.PNG
   :height: 250 px
   :width: 400 px
   :align: center

The descriptor kinematic implies that this model does not consider the mass of the vehicle.
This model captures the trajectory of the vehicle well at low speeds, for which the slip angle is small (i.e. only a few degrees)

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

.. image:: https://github.com/BARCproject/barc/raw/master/docs/imgs/dynBkMdl_eqns.PNG
   :height: 300 px
   :width: 400 px
   :align: center
