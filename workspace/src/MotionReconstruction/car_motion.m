%  Adapted from: Daniel Kawano, Rose-Hulman Institute of Technology
%  Modified by Galaxy Yin

clear all
close all
clc

%%  (1)  Load and process the test data:

load MotionReconstr

%%
% Store the data into the vectors omegaX, omegaY, omegaZ, aX, aY, aZ and
% tdata, which represent the angular velocities, accelerations and
% experiment time sequence from from the IMU topic.

omegaX = sig{1,4}.Data;                 % rad/s
omegaY = sig{1,5}.Data;                 % rad/s
omegaZ = sig{1,6}.Data;                 % rad/s
aX = sig{1,7}.Data;                     % m/s^2
aY = sig{1,8}.Data;                     % m/s^2
aZ = sig{1,9}.Data - sig{1,9}.Data(1);  % m/s^2

tdata = sig{1,1}.Time;

%%  (2)  Use ode45 to numerically integrate the coupled differential
%       equations governing the BARC vehicle's orientation.

%  Simulation parameters:

tol = 1e-6;

psi0 = 0;           %  rad
theta0 = 0;         %  rad
phi0 = 0;           %  rad

Y0 = [psi0, theta0, phi0]';

%  Numerically integrate the state equations in mass-matrix form,
%  M(t,Y)*Y'(t) = F(t,Y):

options = odeset('abstol', tol, 'reltol', tol, 'mass', @M);

[t, Y] = ode45(@F, tdata, Y0, options, omegaX, omegaY, omegaZ, tdata);

%  Extract the results:

psi = Y(:,1);           %  rad
theta = Y(:,2);         %  rad
phi = Y(:,3);           %  rad

%%  (3)  Express the acceleration data in terms of the space-fixed frame by using
%  the computed Euler angles to determine the evolution of the corotational
%  basis {e1,e2,e3}:

%% Rotation
%  ============================Your Code Begins======================
for k = 1:length(psi)
    R1 = TODO
    R2 = TODO
    R3 = TODO
    e1(:,k) = TODO                      %  e1
    e2(:,k) = TODO                      %  e2
    e3(:,k) = TODO                      %  e3
    accel(:,k) = TODO
end
%  ===========================Your Code Ends========================


%% Translation
%  Extract the space-fixed components of acceleration:

Xddot = accel(1,:)';           %  m/s^2
Yddot = accel(2,:)';           %  m/s^2
Zddot = accel(3,:)';           %  m/s^2

%  Use the trapezoidal rule to numerically integrate the acceleration
%  data to determine the displacement of the BARC vehicle.

%  Numerically integrate the acceleration to obtain the BARC vehicle's velocity:

Xdot0 = 0;                     %  m/s
Ydot0 = 0;                     %  m/s
Zdot0 = 0;                     %  m/s

Xdot = cumtrapz(tdata, Xddot) + Xdot0;           %  m/s
Ydot = cumtrapz(tdata, Yddot) + Ydot0;           %  m/s
Zdot = cumtrapz(tdata, Zddot) + Zdot0;           %  m/s

%  Numerically integrate the velocity to obtain the BARC vehicle's displacement:

X0 = 0;        %  m
Y0 = 0;        %  m
Z0 = 0;        %  m

X = cumtrapz(tdata, Xdot) + X0;          %  m
Y = cumtrapz(tdata, Ydot) + Y0;          %  m
Z = cumtrapz(tdata, Zdot) + Z0;          %  m

%%  (4)  Plot and animate the motion of the BARC vehicle.

%  Plot the Euler angles over time:

figure
set(gcf, 'color', 'w')
subplot(311)
plot(t, psi*(180/pi), '-b', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\it\psi\rm: yaw (deg)')
subplot(312)
plot(t, theta*(180/pi), '-r', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\it\theta\rm: pitch (deg)')
subplot(313)
plot(t, phi*(180/pi), '-k', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\it\phi\rm: roll (deg)')

%  Plot the displacement of the BARC vehicle's mass center over time:

figure
set(gcf, 'color', 'w')
subplot(311)
plot(t, X, '-b', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_{1} (m)')
subplot(312)
plot(t, Y, '-r', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_{2} (m)')
subplot(313)
plot(t, Z, '-k', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_{3} (m)')
