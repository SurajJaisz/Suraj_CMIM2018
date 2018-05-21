function [time,u] = odeFE(fun,initialPosition,tspan)
% Forward Euler method for solving ODEs
%   Detailed explanation goes here

time = tspan;
u = zeros(length(initialPosition),length(tspan));
u(:,1) = initialPosition;

for n = 1:length(tspan)-1
    dt = tspan(n+1)-tspan(n);
    u(:,n+1) = u(:,n) + dt*fun(tspan(n),u(:,n));
end

end

