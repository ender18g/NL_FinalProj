function F = f_func(T,psi,q,r,theta,uv)
%F_FUNC
%    F = F_FUNC(T,PSI,Q,R,THETA,UV)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    09-May-2022 15:40:27

t2 = cos(psi);
t3 = cos(theta);
t4 = sin(psi);
t5 = sin(theta);
t6 = q.^2;
t7 = r.^2;
t8 = uv.^2;
t9 = uv.^3;
t10 = t5.^2;
mt1 = [t2.*t3.*t9.*4.6818e-2+t2.*t10.*uv.*2.237-T.*q.*t2.*t5.*(6.11e+2./5.0e+2)-T.*r.*t3.*t4.*(6.11e+2./5.0e+2)-T.*t2.*t3.*uv.*1.86966e-1+q.*t2.*t5.*t8.*(9.01e+2./5.0e+2)+r.*t3.*t4.*t8.*(1.32e+2./1.25e+2)+q.*t2.*t5.*uv.*(1.87e+2./5.0e+2)-t2.*t3.*t6.*uv-t2.*t3.*t7.*uv+q.*r.*t4.*t5.*uv.*2.0];
mt2 = [t3.*t4.*t9.*4.6818e-2+t4.*t10.*uv.*2.237-T.*q.*t4.*t5.*(6.11e+2./5.0e+2)+T.*r.*t2.*t3.*(6.11e+2./5.0e+2)-T.*t3.*t4.*uv.*1.86966e-1+q.*t4.*t5.*t8.*(9.01e+2./5.0e+2)-r.*t2.*t3.*t8.*(1.32e+2./1.25e+2)+q.*t4.*t5.*uv.*(1.87e+2./5.0e+2)-t3.*t4.*t6.*uv-t3.*t4.*t7.*uv-q.*r.*t2.*t5.*uv.*2.0];
mt3 = [t5.*t9.*(-4.6818e-2)-T.*q.*t3.*(6.11e+2./5.0e+2)+T.*t5.*uv.*1.86966e-1+q.*t3.*t8.*(9.01e+2./5.0e+2)+q.*t3.*uv.*(1.87e+2./5.0e+2)+t3.*t5.*uv.*2.237+t5.*t6.*uv];
F = [mt1;mt2;mt3];
