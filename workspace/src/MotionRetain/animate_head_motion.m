%  Daniel Kawano, Rose-Hulman Institute of Technology
%  Last modified:  Mar 30, 2016

function animate_head_motion(x1, x2, x3, e1, e2, e3)

%  For simplicity, represent the head as a sphere:

r = 0.1;                            %  m
[xs, ys, zs] = sphere(36);          %  m
xs = r*xs;                          %  m
ys = r*ys;                          %  m
zs = r*zs;                          %  m

%  Include two points for eyes, which may be located on the head using
%  relative spherical coordinates:

alt = 120*(pi/180);         %  rad
azi = 30*(pi/180);          %  rad

%  Create a circle for drawing lines of longitude and latitude on the
%  head to better visualize the head's change in orientation:

angle = linspace(0, 2*pi, 40)';
circ1 = r*cos(angle);
circ2 = r*sin(angle);

%  The location and orientation of the head vary over time according 
%  to the calculated motion of the mass center and the evolution of the 
%  corotational basis {e1,e2,e3}:

for k = 1:length(x1)
    xcirc1(:,k) = x1(k) + circ1*e1(1,k) + circ2*e2(1,k);        %  m
    ycirc1(:,k) = x2(k) + circ1*e1(2,k) + circ2*e2(2,k);        %  m
    zcirc1(:,k) = x3(k) + circ1*e1(3,k) + circ2*e2(3,k);        %  m
    xcirc2(:,k) = x1(k) + circ1*e2(1,k) + circ2*e3(1,k);        %  m
    ycirc2(:,k) = x2(k) + circ1*e2(2,k) + circ2*e3(2,k);        %  m
    zcirc2(:,k) = x3(k) + circ1*e2(3,k) + circ2*e3(3,k);        %  m
    xcirc3(:,k) = x1(k) + circ1*e1(1,k) + circ2*e3(1,k);        %  m
    ycirc3(:,k) = x2(k) + circ1*e1(2,k) + circ2*e3(2,k);        %  m
    zcirc3(:,k) = x3(k) + circ1*e1(3,k) + circ2*e3(3,k);        %  m
    rrighteye(:,k) = [x1(k), x2(k), x3(k)]' + ...               %  m
                     r*sin(alt)*cos(+azi)*e1(:,k) + ...
                     r*sin(alt)*sin(+azi)*e2(:,k) + ...
                     r*cos(alt)*e3(:,k);
    rlefteye(:,k) = [x1(k), x2(k), x3(k)]' + ...                %  m
                    r*sin(alt)*cos(-azi)*e1(:,k) + ...
                    r*sin(alt)*sin(-azi)*e2(:,k) + ...
                    r*cos(alt)*e3(:,k);
end

%  Set up the figure window:

figure
set(gcf, 'color', 'w')
head = surf('xdata', xs, 'ydata', ys, 'zdata', zs, 'facealpha', 1, 'edgealpha', 1);
colormap([1, 0.85, 0.70])
shading interp
xlabel('\itx\rm_{1} (m)')
set(gca, 'xdir', 'reverse')
ylabel('\itx\rm_{2} (m)')
zlabel('\itx\rm_{3} (m)            ', 'rotation', 0)
set(gca, 'zdir', 'reverse')
axis equal
xlim([min(x1)-1.2*r, max(x1)+1.2*r])
ylim([min(x2)-1.2*r, max(x2)+1.2*r])
zlim([min(x3)-1.2*r, max(x3)+1.2*r])
grid on

%  Locate and orient the lines of longitude and latitude appropriately, and
%  keep track of the eyes:

circle1 = line('xdata', xcirc1(:,1), 'ydata', ycirc1(:,1), 'zdata', zcirc1(:,1), 'color', 'b', 'linewidth', 3);
circle2 = line('xdata', xcirc2(:,1), 'ydata', ycirc2(:,1), 'zdata', zcirc2(:,1), 'color', 'r', 'linewidth', 3);
circle3 = line('xdata', xcirc3(:,1), 'ydata', ycirc3(:,1), 'zdata', zcirc3(:,1), 'color', 'k', 'linewidth', 3);
righteye = line('xdata', rrighteye(1,1), 'ydata', rrighteye(2,1), 'zdata', rrighteye(3,1), 'marker', 'o', 'color', 'k', 'markerfacecolor', 'k', 'linewidth', 3);
lefteye = line('xdata', rlefteye(1,1), 'ydata', rlefteye(2,1), 'zdata', rlefteye(3,1), 'marker', 'o', 'color', 'k', 'markerfacecolor', 'k', 'linewidth', 3);

%  Animate the head's motion by updating the figure with its current
%  location and orientation:

pause

% animation = VideoWriter('head-motion.avi');
% animation.FrameRate = 100;
% open(animation);

for k = 1:5:length(x1)
    set(head, 'xdata', xs + x1(k), 'ydata', ys + x2(k), 'zdata', zs + x3(k));
    set(circle1, 'xdata', xcirc1(:,k), 'ydata', ycirc1(:,k), 'zdata', zcirc1(:,k));
    set(circle2, 'xdata', xcirc2(:,k), 'ydata', ycirc2(:,k), 'zdata', zcirc2(:,k));
    set(circle3, 'xdata', xcirc3(:,k), 'ydata', ycirc3(:,k), 'zdata', zcirc3(:,k));
    set(righteye, 'xdata', rrighteye(1,k), 'ydata', rrighteye(2,k), 'zdata', rrighteye(3,k));
    set(lefteye, 'xdata', rlefteye(1,k), 'ydata', rlefteye(2,k), 'zdata', rlefteye(3,k));
    drawnow   
    % writeVideo(animation, getframe(gcf));
end

% close(animation);