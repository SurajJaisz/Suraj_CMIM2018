function [time,y] = odeRK4(fun,tspan,y0)
% odeRK4  Fourth order Runge-Kutta method for a single, first order ODE

time = tspan;
n = length(time);
y = zeros(length(y0),n);
y(:,1) = y0;

for i = 1:n-1
    h = tspan(i+1)-tspan(i);
    k1 = h*fun(tspan(i),y(:,i));
    k2 = h*fun(tspan(i)+0.5*h,y(:,i)+0.5*k1);
    k3 = h*fun(tspan(i)+0.5*h,y(:,i)+0.5*k2);
    k4 = h*fun(tspan(i)+h,y(:,i)+k3);
    y(:,i+1) = y(:,i)+(1/6)*k1+(1/3)*k2+(1/3)*k3+(1/6)*k4;
end