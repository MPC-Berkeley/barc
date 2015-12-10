% @author: jon gonzales
% this program fits curve to the servo steering angle to servo signal value
clc;
clear;

% x := input servo signal value [PWM]
% y := output servo angle [deg]
x = 30:5:120;
y = 0.00000488*x.^3 - 0.00247009*x.^2 - 0.22553884*x + 37.690229;

n = numel(x);

% linear fit
% A = [ones(n,1), y'];
% sol = (A'*A)\A'*x';
% a0 = sol(1);
% a1 = sol(2);

% polynomial fit
A = [ones(n,1), y' , (y.^2)'];
sol = (A'*A)\A'*x';
a0 = sol(1);
a1 = sol(2);
a2 = sol(3);

% polynomial fit from PWM signal to steering angle
A = [ones(n,1), x' , (x.^2)'];
sol = (A'*A)\A'*y';
b0 = sol(1);
b1 = sol(2);
b2 = sol(3);

% plot results
figure;
plot(y,x,'*-'); hold on;
plot(y,a0 + a1*y + a2*y.^2,'-');
legend('measured','poly interp')
xlabel('servo angle $$\delta $$ [deg]','interpreter','latex')
ylabel('servo input signal $$ u $$ [PWM]','interpreter','latex')
title('Servo angle to PWM signal mapping','interpreter','latex')
grid on;

figure;
plot(x,y,'*-'); hold on;
plot(x,b0 + b1*x + b2*x.^2,'-');
legend('measured','poly interp')
xlabel('servo input signal $$ u $$ [PWM]','interpreter','latex')
ylabel('servo angle $$\delta $$ [deg]','interpreter','latex')
title('PWM signal to servo angle mapping','interpreter','latex')
grid on;
