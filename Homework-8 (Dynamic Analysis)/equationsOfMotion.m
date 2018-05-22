function ydot = equationsOfMotion(time,position,InputParameters)
% (InputParameters,position,time)
% Equations of motion of the System
%   Detailed explanation goes here

%% Separating position(y) and velocity(yd)
s = 3*numel(InputParameters.bodies);
y = position(1:s,1);
yd = position(s+1:2*s,1);

%% Calculating the Mass Matrix and Force Vector
M = systemMassMatrix(InputParameters); % Calculating total mass matrix of the system
Q = systemForceVector(InputParameters); % Calculating total force vector of the system

%% Calculating T and Cdot
Qc = C_Gamma(InputParameters,y,yd,time);
if InputParameters.numConsEqn > 0
    Cdot = JacobianMatrix(InputParameters,y)*yd + Ct(InputParameters,time);
else
    Cdot = [];
end

%% Equations of Motion
LHS = [                 M,                  JacobianMatrix(InputParameters,y)';
       JacobianMatrix(InputParameters,y),   zeros(length(Cdot))];

RHS = [Q;
       Qc-2*InputParameters.alpha*Cdot-InputParameters.beta^2*constraintEquations(InputParameters,y,time)];

ydot = [yd; LHS\RHS];
end

