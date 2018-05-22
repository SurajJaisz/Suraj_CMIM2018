function [time,q,qd,qdd] = kinematicAnalysis(InputParameters)
% Kinematic Analysis of the system is done here.
%   Detailed explanation goes here.

time = InputParameters.tspan;
dt = InputParameters.tstep;

%% Defining Initial Values of the Position, Velocity, and Acceleration Coordinate (2D System)

q0 = zeros(3*numel(InputParameters.bodies),1);
for n = 1:numel(InputParameters.bodies)
    q0(3*(n-1)+1:3*n) = InputParameters.bodies(n).position;
end
q = zeros(length(q0),length(time));
qd = zeros(length(q0),length(time));
qdd = zeros(length(q0),length(time));

%% Solution for each TimeStep
for t = 1:length(time)
    tolerance = 1e-6; % Parameters for Newton-Raphson method
    maxIterations = 1000; % Parameters for Newton-Raphson method
    h = 1e-4;
    
    % Solution for Position Analysis
    if t ==1 % No solution intended in that situation
        q(:,t) = q0;
        x0d = zeros(3*numel(InputParameters.bodies),1);
        x0dd = zeros(3*numel(InputParameters.bodies),1);        
    else
%         x0 = q(:,t-1);
%         x0d = qd(:,t-1);
%         x0dd = qdd(:,t-1);
        % Taylor expansion to determine initial guess
        x0 = q(:,t-1) + qd(:,t-1)*dt + 0.5*qdd(:,t-1)*dt^2;
        
        constEqs = @(x) constraintEquations(InputParameters,x,time(t));
        jacobFDiff = @(x) jacobianFDifference(constEqs,x,h);
%         q(:,t) = Newton_Raphson(constEqs,jacobDiff,x0,tolerance);
        q(:,t) = Newton_Raphson(constEqs,jacobFDiff,x0,tolerance,maxIterations);
    end
    
    % Solution for Velocity Analysis
    constEqsD = @(xd) JacobianMatrix(InputParameters,q(:,t))*xd + Ct(InputParameters,q(:,t),xd,time(t));
    jacobFDiffD = @(xd) jacobianFDifference(constEqsD,xd,h);
%     qd(:,t) = Newton_Raphson(constEqsD,jacobDiffD,x0d,tolerance);
    qd(:,t) = Newton_Raphson(constEqsD,jacobFDiffD,x0d,tolerance,maxIterations);
    
    % Solution for Acceleration Analysis
    constEqsDD = @(xdd) JacobianMatrix(InputParameters,q(:,t))*xdd - C_Gamma(InputParameters,q(:,t),qd(:,t),time(t));
    jacobFDiffDD = @(xdd) jacobianFDifference(constEqsDD,xdd,h);
%     qdd(:,t) = Newton_Raphson(constEqsDD,jacobDiffDD,x0dd,tolerance);
    qdd(:,t) = Newton_Raphson(constEqsDD,jacobFDiffDD,x0dd,tolerance,maxIterations);

end