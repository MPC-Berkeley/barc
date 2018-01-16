%  Adapted from: Daniel Kawano, Rose-Hulman Institute of Technology
%  Modified by Galaxy Yin

function dY = F(t, Y, omega1, omega2, omega3, tdata)

%  Interpolate the angular velocity data to obtain the values needed to
%  evaluate the state equations at the current integration step:

omega1 = interp1(tdata, omega1, t);
omega2 = interp1(tdata, omega2, t);
omega3 = interp1(tdata, omega3, t);

%  Define the right-hand side of the state equations, F(t,Y):

dY = [omega1;
      omega2;
      omega3];
