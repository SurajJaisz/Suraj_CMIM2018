function [t,y] = eulode(dydt, tspan, y0, h)
%eulode: Euler ODE solver
%   [t,y] = eulode(dydt, tspan, y0, h, p1, p2,...)
%   `   uses EULER'S method to INTEGRATE an ODE
%       (uses the slope at the beginning of the stepsize to graph the
%       function.)
%Input:
%   dydt    = name of hte M-file that evaluates the ODE
%   tspan   = [ti,tf] where ti and tf = initial and final values of
%               independent variables
%   y0      = initial value of dependent variable
%   h       = step size
%   p1,p2   = additional parameter used by dydt
%Output:
%   t = vector of independent variable
%   y = vector of solution for dependent variable

if nargin<4, error('at least 4 input arguments required'), end
ti = tspan(1); tf = tspan(2);
if ~ (tf>ti), error('upper limit must be greater than lower limit'), end

t = (ti:h:tf)'; 
n = length(t);
%if necessary, add an additional value of t 
%so that range goes from t=ti to tf

if t(n)<tf
    t(n+1) = tf;
    n = n+1;
    t(n)=tf;
end

y = y0*ones(n,1); %preallocate y to improve efficiency

for i = 1:n-1 %implement Euler's Method
    y(i+1) = y(i) + dydt(t(i),y(i))*(t(i+1)-t(i));
end

plot(t,y)


% 
% h = 0.1 % Step Size
% N = 10 % Number of steps
% y(1) = 1;
% 
% for n=1:N
%     y(n+1)=y(n)+h*(-6*y(n));
%     x(n+1)=n*h;
% end
% plot(x,y)
