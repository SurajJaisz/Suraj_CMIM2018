function ydot = equationsOfMotion(InputParameters,position,time)
% Equations of motion of the System
%   Detailed explanation goes here

%% Separating position(y) and velocity(yd)
y = position(1:InputParameters.numGenCood,1);
yd = position((InputParameters.numGenCood+1:2*InputParameters.numGenCood),1);

%% Calculating the Mass Matrix and Force Vector
M = massMatrix(InputParameters,time); % Calculating total mass matrix
Q = forceVector(InputParameters,time); % Calculating total force vector

%% Calculating T and Cdot
T = C_Gamma(InputParameters,y,yd,time)
if

%% Equations of Motion
LHS = [M, JacobianMatrix(InputParameters,y)';
       JacobianMatrix(InputParameters,y), zeros(length(something))];

RHS = [Q;
       T-2*InputParameters.alpha*Cdot-InputParameters.beta^2*constraintEquations(InputParameters,y,time)];

ydot = [yd; LHS\RHS];
end

