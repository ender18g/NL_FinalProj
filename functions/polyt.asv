%generate a lambda vector (time, deg of t^deg, diff times)
function output = polyt(T,deg,dt)

syms t;
t_vector = t* ones(deg+1,1);

%loop through and find eac value of t
for i = 1:length(t_vector)
    t_vector(i)=t^(deg+1-i);
end

%differentiate (0 is no differentiation)
for i=1:dt
    t_vector = diff(t_vector,t);
end
%finally evaluate at t (or array of t)
t=T;
output = subs(t_vector);
end