function out = TrajFollow(x0,xf,T,current)
%pass in x0, xf and time

%FeedbackLinCalc;
%add functions path
addpath('functions');
tic; 
%define the size of our working area
box_size = 200;
%time of trajectory

%number of obstacles
num_obst = 0;
% boundary conditions in state space
% x y z theta psi uv q r t


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
%safe the desired path
out.X_des = double(X);

%% plot desired path
figure(3);
plot3(X(1,:), X(2,:), X(3,:), '--r',LineWidth=2)
hold on;
title("AUV Trajectory Following")
grid on;
ylabel("y");
xlabel("x");
zlabel("z");
xlim([-box_size box_size])
ylim([-box_size box_size])
subtitle("Using Feedback Linearization")

%% Plot AUV actual path
tspan = [0 T];


Opt =odeset('Events',@(t,x) eventsFcn(t,x,T,xf) );
[t,x,te,ye,ie] = ode45(@(t,x) AUVdynamics(t,x,current),tspan,x0,Opt);


out.t_act = t;
out.x_act = x;
plot3(x(:,1),x(:,2),x(:,3),Color='blue',LineWidth=1)
hold on;

% add in obstacles
for i=1:num_obst
    obst_size = 1;
    gen_obstacle(obst_size,box_size*rand(3,1));
end
legend("Desired", "Actual")

out.frame = getframe(3);


%display the total time
toc
end

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