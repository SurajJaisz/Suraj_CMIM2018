function [t,q,qd,lambda] = dynamicAnalysis(InputParameters)
% Dynamic Analysis of the system is done here.
%   Detailed explanation goes here

timespan = InputParameters.tspan;

%% Defining Initial Conditions (2D System)

x0 = zeros(InputParameters.numGenCood,1);

for n = 1:numel(InputParameters.bodies)
    x0(3*(n-1)+1:3*n) = InputParameters.bodies(n).position;
end
q0 = [x0;zeros(size(x0));zeros(InputParameters.numConsEqn)];
% q0 = [0;0;0;0;0;0;0;0;0];
% qd = zeros(length(q0),length(time));
% qdd = zeros(length(q0),length(time));

%% Solution for Dynamic Analysis

if strcmp(InputParameters.solver,'ode45')
    [t,y] = ode45(@equationsOfMotion,timespan,q0,InputParameters.solverOptn,InputParameters);
end
q = y(:,1:InputParameters.numGenCood)';
qd = y(:,(InputParameters.numGenCood+1):(2*InputParameters.numGenCood))';
lambda = y(:,(2*InputParameters.numGenCood+1):end)';

end

