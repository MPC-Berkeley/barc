%  Daniel Kawano, Rose-Hulman Institute of Technology
%  Last modified:  Mar 30, 2016

clear all
close all
clc

%%  (1)  Load and process the crash test data:

load imu

% store the data and experiment time sequence

omega1 = states(:,1);
omega2 = states(:,2);
omega3 = states(:,3);
a1 = states(:,4);
a2 = states(:,5);
a3 = states(:,6);

tdata = time;


%%  (2)  Use ode45 to numerically integrate the coupled differential
%       equations governing the head's orientation.

%  Simulation parameters:

tol = 1e-6;

psi0 = 0;           %  rad
theta0 = 0;         %  rad
phi0 = 0;           %  rad

Y0 = [psi0, theta0, phi0]';

%  Numerically integrate the state equations in mass-matrix form,
%  M(t,Y)*Y'(t) = F(t,Y):

options = odeset('abstol', tol, 'reltol', tol, 'mass', @M);

[t, Y] = ode45(@F, tdata, Y0, options, omega1, omega2, omega3, tdata);

%  Extract the results:

psi = Y(:,1);           %  rad
theta = Y(:,2);         %  rad
phi = Y(:,3);           %  rad

%%  (3)  Use the trapezoidal rule to numerically integrate the acceleration
%       data to determine the displacement of the head.

%  Express the acceleration data in terms of the space-fixed frame by using
%  the computed Euler angles to determine the evolution of the corotational
%  basis {e1,e2,e3}:

for k = 1:length(psi)
    R1 = [1, 0, 0;
        0, cos(phi(k)), sin(phi(k));
        0, -sin(phi(k)), cos(phi(k))];
    R2 = [cos(theta(k)), 0, -sin(theta(k));
        0, 1, 0;
        sin(theta(k)), 0, cos(theta(k))];     
    R3 = [cos(psi(k)), sin(psi(k)), 0;          
        -sin(psi(k)), cos(psi(k)), 0;           
        0, 0, 1];
    e1(:,k) = ([1, 0, 0]*(R1*R2*R3))';                      %  e1
    e2(:,k) = ([0, 1, 0]*(R1*R2*R3))';                      %  e2
    e3(:,k) = ([0, 0, 1]*(R1*R2*R3))';                      %  e3
    accel(:,k) = a1(k)*e1(:,k) + a2(k)*e2(:,k) + ...        %  m/s^2
        a3(k)*e3(:,k);
end

%  Extract the space-fixed components of acceleration:

x1ddot = accel(1,:)';                        %  m/s^2
x2ddot = accel(2,:)';                        %  m/s^2
x3ddot = accel(3,:)' - accel(3,1);           %  m/s^2

%  Numerically integrate the acceleration to obtain the head's velocity:

x1dot0 = 0;          %  m/s
x2dot0 = 0;                     %  m/s
x3dot0 = 0;                     %  m/s

x1dot = cumtrapz(tdata, x1ddot) + x1dot0;           %  m/s
x2dot = cumtrapz(tdata, x2ddot) + x2dot0;           %  m/s
x3dot = cumtrapz(tdata, x3ddot) + x3dot0;           %  m/s

%  Numerically integrate the velocity to obtain the head's displacement:

x10 = 0;        %  m
x20 = 0;        %  m
x30 = 0;        %  m

x1 = cumtrapz(tdata, x1dot) + x10;          %  m
x2 = cumtrapz(tdata, x2dot) + x20;          %  m
x3 = cumtrapz(tdata, x3dot) + x30;          %  m

%%  (4)  Plot and animate the motion of the head.

%  Plot the Euler angles over time:

figure
set(gcf, 'color', 'w')
subplot(311)
plot(t, psi*(180/pi), '-b', 'linewidth', 2)
xlabel('Time (ms)')
ylabel('\it\psi\rm (deg)')
subplot(312)
plot(t, theta*(180/pi), '-r', 'linewidth', 2)
xlabel('Time (ms)')
ylabel('\it\theta\rm (deg)')
subplot(313)
plot(t, phi*(180/pi), '-k', 'linewidth', 2)
xlabel('Time (ms)')
ylabel('\it\phi\rm (deg)')

%  Plot the displacement of the head's mass center over time:

figure
set(gcf, 'color', 'w')
subplot(311)
plot(t, x1, '-b', 'linewidth', 2)
xlabel('Time (ms)')
ylabel('\itx\rm_{1} (m)')
subplot(312)
plot(t, x2, '-r', 'linewidth', 2)
xlabel('Time (ms)')
ylabel('\itx\rm_{2} (m)')
subplot(313)
plot(t, x3, '-k', 'linewidth', 2)
xlabel('Time (ms)')
ylabel('\itx\rm_{3} (m)')

%%  Animate the head's motion:

animate_motion(x1, x2, x3, e1, e2, e3);
