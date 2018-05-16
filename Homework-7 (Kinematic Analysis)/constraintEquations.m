function [constraintMatrix] = constraintEquations(InputParameters,position,time)
%The Constraint Equations of the System
%   Detailed explanation goes here
constraintMatrix = [];

%% Defining Constraint Equations for Joints (only Revolute Joint)
for n = 1:numel(InputParameters.joints)
    jointType = InputParameters.joints(n).type;
    jointBodies = InputParameters.joints(n).bodies;
    jointLocation = InputParameters.joints(n).location;
    
    if jointBodies(1) == 0
        bodyPosition = [[0 0 0]' position(3*(numel(InputParameters.bodies(2))-1)+1:3*numel(InputParameters.bodies(2)))];
    elseif jointBodies(2) == 0
        bodyPosition = [position(3*(numel(InputParameters.bodies(1))-1)+1:3*numel(InputParameters.bodies(1))) [0 0 0]'];
    else
        bodyPosition = [position(3*(numel(InputParameters.bodies(1))-1)+1:3*numel(InputParameters.bodies(1))) position(3*(numel(InputParameters.bodies(2))-1)+1:3*numel(InputParameters.bodies(2)))];
    end
    
    if strcmp(jointType,'revolute')
        constraintMatrix = [constraintMatrix
                            bodyPosition(1,1) - bodyPosition(1,2) + jointLocation(1,1)*cos(bodyPosition(3,1)) - jointLocation(1,2)*cos(bodyPosition(3,2)) - jointLocation(2,1)*sin(bodyPosition(3,1)) + jointLocation(2,2)*sin(bodyPosition(3,2))
                            bodyPosition(2,1) - bodyPosition(2,2) + jointLocation(1,1)*sin(bodyPosition(3,1)) - jointLocation(1,2)*sin(bodyPosition(3,2)) + jointLocation(2,1)*cos(bodyPosition(3,1)) - jointLocation(2,2)*cos(bodyPosition(3,2))];
    end
end

%% Defining Time Dependent Constraint Equations
for s = 1:numel(InputParameters.timeConstraints)
    bodyDOF = 3*(numel(InputParameters.timeConstraints(s).body)-1)+1;
    constraintDOF = bodyDOF(InputParameters.timeConstraints(s).DOF);
    constraintExpression = InputParameters.timeConstraints(s).fun;
    
    constraintMatrix = [constraintMatrix constraintExpression(time)-x(constraintDOF)]';
end

end