function dx = AUVdynamics(t,x,current)
%ODE Function to simulate AUV dynamics

%load in model paramters
AUVparameters;
%create a dx vector with zeros
dx = zeros (9,1);

% get the controls:
d_c = FbLinCtrl(t,x);
% assign input variables
d_q = d_c(1);
d_r = d_c(2);
d_u = d_c(3);

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


% limit thrust 0 to 1
if T>5
    T = 5;
end

if T<-5
    T=-5;
end


% limit uv 0 to 2
if uv>5
    uv = 5;
end

if uv<0
    uv=0;
end



%Calculate the change of each state
dx(1)= uv*cos(theta)*cos(psi);
dx(2)= uv*cos(theta)*sin(psi);
dx(3)= -uv*sin(theta);
dx(4) = q;
dx(5) = r;
dx(6) = Xuu * uv^2 + a * T;
dx(7) = Muq * uv * q + Mq * q - B * sin(theta)+ b * uv^2 * d_q;
dx(8) = Nur * uv*r + c*uv^2*d_r;
dx(9) = d_u;




%add in current
dx(1:3) = dx(1:3)+current;

end

