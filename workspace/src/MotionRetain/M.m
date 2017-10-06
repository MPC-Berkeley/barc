%  Daniel Kawano, Rose-Hulman Institute of Technology
%  Last modified:  Mar 30, 2016

function mass = M(t, Y, omega1, omega2, omega3, tdata)

%  For convenience, define the state variables:

psi = Y(1);
theta = Y(2);
phi = Y(3);

%  Construct the mass matrix, M(t,Y):

mass = [-sin(theta), 0, 1;
        cos(theta)*sin(phi), cos(phi), 0;
        cos(theta)*cos(phi), -sin(phi), 0];