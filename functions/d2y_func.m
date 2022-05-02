function d2Y = d2y_func(T,psi,q,r,theta,uv)
%D2Y_FUNC
%    d2Y = D2Y_FUNC(T,PSI,Q,R,THETA,UV)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    01-May-2022 17:47:21

t2 = cos(psi);
t3 = cos(theta);
t4 = sin(psi);
t5 = sin(theta);
t6 = uv.^2;
t7 = T.*(6.11e+2./1.0e+3);
t8 = t6.*(1.53e+2./1.0e+3);
t9 = -t8;
t10 = t7+t9;
d2Y = [t2.*t3.*t10-q.*t2.*t5.*uv-r.*t3.*t4.*uv;t3.*t4.*t10-q.*t4.*t5.*uv+r.*t2.*t3.*uv;-t5.*t10-q.*t3.*uv];
