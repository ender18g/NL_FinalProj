function input_vec = FbLinCtrl(t,x)
% Determine the input parameters for trajectory following

%load ctrl parameters
load("FbLinParams.mat");

%unpack state variables
px = x(1);
py = x(2);
pz = x(3);
theta = x(4);
psi = x(5);
uv=x(6);
q=x(7);
r=x(8);
T = x(9);

%get useful dynamics
dx = zeros(3,1);
dx(1)= uv*cos(theta)*cos(psi);
dx(2)= uv*cos(theta)*sin(psi);
dx(3)= -uv*sin(theta);

%define Gains
K=1*[1; 2; 2];



%get error states
z={};
z{1} = x(1:3)-A*poly0(t);
z{2} = dx-A*poly1(t);
z{3} = d2y_func(T,psi,q,r,theta,uv)-A*poly2(t);

% calculate v
% v = yd3d - Ki(yi-ydi)
v = A*poly3(t)-K(1)*z{1}- K(2)*z{2}-K(3)*z{3};

% calculate inputs
inv_G = ig_func(psi,theta,uv);
F = f_func(T,psi,q,r,theta,uv);

input_vec = inv_G*(v-F)

end
