%clean up workspace
clearvars; close all; clc;
%perform necessary calculations / function generation
FeedbackLinCalc;
%add functions path
addpath('functions');
tic; 
%define the size of our working area
box_size = 10;
%time of trajectory
T = 60;
%number of obstacles
num_obst = 0;
% boundary conditions in state space
% x y z theta psi uv q r t
x0 = [0 0 0 0 0 .5 0 0 .1]';
xf = [50 50 5 0 0 .5 0 0 0]';

%define current
current = 0* [1 0 0]';

% perturb the system IC with the offset:
ic_offset = [10*[0,-1,-1/5]';zeros(6,1)];

%%%%%%%%% TRAJECTORY GENERATION %%%%%%%%%%%%%
% boundary conditions in flat output space
y0 = auv_h(x0);
yf = auv_h(xf);
%starting and ending velocities / accelrations
dy0 = 1*[x0(6) *cos(x0(4))*cos(x0(5));x0(6) *cos(x0(4))*sin(x0(5));-x0(6)*sin(x0(4))];
dyf = 1*[xf(6) *cos(xf(4))*cos(xf(5));xf(6) *cos(xf(4))*sin(xf(5));-xf(6)*sin(xf(4))];
d2y0 = 0 * [0;0;0]; 
d2yf = 0 * [0;0;0]; 

%% compute path coefficients
A = poly3_coeff(y0, dy0, d2y0, yf, dyf, d2yf, T,5);
% save A for controller
save("FbLinParams.mat","A");
% create path X
X = A * polyt(0:.01:T,5,0);

%% plot desired path
figure(1)
subplot(3,1,1);
plot3(X(1,:), X(2,:), X(3,:), '--r',LineWidth=1.5)
hold on;
title("3D AUV Trajectory Following w Feedback Linearization")
grid on;
ylabel("y (meters)");
xlabel("x (meters)");
zlabel("z (meters)");
subtitle(sprintf("Initial Cond Offset = [%.1f,%.1f,%.1f] m",ic_offset(1:3)))
subtitle(sprintf("Current = [%.1f,%.1f,%.1f] m/s",current))

%% Plot AUV actual path
tspan = [0 T];
[t,x] = ode45(@(t,x) AUVdynamics(t,x,current),tspan,x0+ic_offset);
plot3(x(:,1),x(:,2),x(:,3),Color='blue',LineWidth=1.5)

% add in obstacles
for i=1:num_obst
    obst_size = 1;
    gen_obstacle(obst_size,box_size*rand(3,1));
end
legend("Desired Trajectory", "Actual Trajectory")

%% plot control signal
subplot(3,1,2);
u = zeros(3,length(t));
for i=1:length(t)
u(:,i)=FbLinCtrl(t(i),x(i,:)');
end

plot (t,u);
grid on;
title('Feedback Linearization Control Signal');
subtitle('With bounded control inputs')
legend('d_q','d_r','d_u');
xlabel('Time (seconds)')
ylabel('Control Signal')


%% plot error
% generate error vector
traj_des = double(A* polyt(t',5,0));
error = x(:,1:3) - traj_des(1:3,:)';

norm_e = zeros(length(t),1);

for i=1:length(t)
norm_e(i) = norm(error(i));
end


figure(1)
subplot(3,1,3);
plot(t,norm_e, 'blue',LineWidth=1.5)
hold on;
title("Norm of Positional Error vs Time")
grid on;
ylabel("Error Norm (meters)");
xlabel("Time (seconds)");


%display the total time
toc
function A = poly3_coeff(y0, dy0, d2y0, yf, dyf, d2yf, T,deg)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T
Y = [y0, dy0, d2y0, yf, dyf d2yf];
L = [polyt(0,deg,0), polyt(0,deg,1), polyt(0,deg,2)...
    polyt(T,deg,0), polyt(T,deg,1), polyt(T,deg,2)];
A = Y/L;
A = double(A);
end

function f = auv_h(x)
f = x(1:3);
end

function f = gen_obstacle(r,pos)
%define variables XYZ
[X, Y, Z] = sphere;

%Create sphere vector
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;

%plot sphere
sphereObj = surf(X2+pos(1),Y2+pos(2),Z2+pos(3));
set(sphereObj,'edgecolor', 'none');

end