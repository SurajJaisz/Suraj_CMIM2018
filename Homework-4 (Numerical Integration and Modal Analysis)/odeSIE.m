function [time,u,v] = odeSIE(funF,funG,tspan,u0,v0)
%odeSIE Simple integration method using Semi-implicit Euler
%   Detailed explanation goes here
time = tspan;
n = length(time);
u = zeros(length(u0),n);
v = zeros(length(v0),n);
u(:,1) = u0;
v(:,1) = v0;

for i = 1:n-1
    dt = tspan(i+1)-tspan(i);
    v(:,n+1) = v(:,n) + dt*funG(tspan(n),u(:,n));
    u(:,n+1) = u(:,n) + dt*funF(tspan(n),v(:,n+1));
end

