clc; clearvars;

addpath('functions');

% Define time dependent eqns:
syms  uv_t(t) theta_t(t) psi_t(t) q_t(t) r_t(t) T_t(t);

% Define constants
syms Xuu a Muq Mq B b Nur c x t;
% Define inputs
syms d_u d_r d_q;

input_vec = [d_q;d_r;d_u];

% equations of motion
dy1 = uv_t * cos(theta_t) * cos(psi_t);
dy2 = uv_t * cos(theta_t) * sin(psi_t);
dy3 = -uv_t *sin (theta_t);

%extended state dynamics
theta_dot = q_t;
psi_dot = r_t;
uv_dot = Xuu * uv_t^2 + a * T_t;
q_dot = Muq * uv_t * q_t + Mq * q_t - B * sin(theta_t)+ b * uv_t^2 * d_q;
r_dot = Nur * uv_t*r_t + c*uv_t^2*d_r;
T_dot = d_u;

% make arrays for substitutions
vars = [theta_t(t),psi_t(t),uv_t(t),q_t(t),r_t(t),T_t(t)];
state_dyn = {'theta_dot','psi_dot','uv_dot','q_dot','r_dot','T_dot'};

% create a vector dY
dY = [dy1;dy2;dy3];


d2Y= diff(dY,t);
% substitute all dynamics
for i=1:numel(state_dyn)
    char_var = char(vars(i));
    char_sd = state_dyn{i};
    eval_str = sprintf('subs(d2Y,diff(%s,t),%s);',char_var,char_sd);
    d2Y = eval(eval_str);
end
% Simplify output
d2Y = simplify(d2Y);


d3Y = diff(d2Y,t);
% substitute all dynamics
for i=1:numel(state_dyn)
    char_var = char(vars(i));
    char_sd = state_dyn{i};
    eval_str = sprintf('subs(d3Y,diff(%s,t),%s);',char_var,char_sd);
    d3Y = eval(eval_str);
end
% Simplify output
d3Y = simplify(d3Y);

syms theta psi uv q r T

%replace functions that have time dependence
Yders = {dY,d2Y,d3Y};
vars = [theta_t(t),psi_t(t),uv_t(t),q_t(t),r_t(t),T_t(t)];
states = [theta, psi, uv,q,r,T];
%states = [x(4),x(5), x(6),x(7),x(8),x(9)];

%replace variables with state names
for i=1:numel(Yders)
    for j=1:numel(vars)
        Yders{i} = subs(Yders{i},vars(j),states(j));
    end
    %simplify and remove t dependence
    Yders{i}=simplify(Yders{i});
    Yders{i} = Yders{i}(t);
end

% Dissect system:  d3Y = F + G u
G = jacobian(Yders{3},input_vec);
F = Yders{3} - (G*input_vec);
F = simplify(F);
G = simplify(G);

% if F has inputs in it (d_u d_q d_r) get angry:
if (has(F,d_u))
    fprintf(2,"*** Jacobian DOESN'T Fing WORK***")
end

% check to ensure d3y = F + G u
err_ck = simplify(Yders{3} - (F + G*input_vec));
if (err_ck~=0)
    fprintf(2,"*** Jacobian DOESN'T Fing WORK***")
end

%get the latest auv parameters
AUVparameters;

%substitue them into the varialbes that we need
inv_G = subs(inv(G));
F = subs(F);
d2Y = subs(Yders{2});

poly={};
for i=0:3
poly{i+1} = polyt(t,5,i);
end


matlabFunction(d2Y,"File","functions/d2y_func");
matlabFunction(F,"File","functions/f_func");
matlabFunction(inv_G,"File","functions/ig_func");
matlabFunction(poly{1},"File","functions/poly0");
matlabFunction(poly{2},"File","functions/poly1");
matlabFunction(poly{3},"File","functions/poly2");
matlabFunction(poly{4},"File","functions/poly3");


