function constraintEquations = constraintEquations(InputParameters,position,time)
%The Constraint Equations of the System
%   Detailed explanation goes here
constraintEquations = [];

%% Defining Constraint Equations for Joints (only Revolute Joint)
for n = 1:numel(InputParameters.joints)
    jointType = InputParameters.joints{n}.type;
    jointBodies = InputParameters.joints{n}.bodies;  

    if jointBodies(1) == 0
        bodyPosition = [[0;0;0], position(rangeCal(jointBodies(2)))];
    elseif jointBodies(2) == 0
        bodyPosition = [position(rangeCal(jointBodies(1))), [0;0;0]];
    else
        bodyPosition = [position(rangeCal(jointBodies(1))), position(rangeCal(jointBodies(2)))];
    end
    
    
    if strcmp(jointType,'revolute')
        jointLocation = InputParameters.joints{n}.location;
        
        constraintEquations = [constraintEquations
                                bodyPosition(1,1)-bodyPosition(1,2)+jointLocation(1,1)*cos(bodyPosition(3,1))-jointLocation(1,2)*cos(bodyPosition(3,2))-jointLocation(2,1)*sin(bodyPosition(3,1))+jointLocation(2,2)*sin(bodyPosition(3,2))
                                bodyPosition(2,1)-bodyPosition(2,2)+jointLocation(1,1)*sin(bodyPosition(3,1))-jointLocation(1,2)*sin(bodyPosition(3,2))+jointLocation(2,1)*cos(bodyPosition(3,1))-jointLocation(2,2)*cos(bodyPosition(3,2))];
    elseif strcmp(jointType,'translational')
        jointLocation = InputParameters.joints{n}.location;
        jointNormal = InputParameters.joints{n}.normal;
        bodyPosition0 = InputParameters.joints{n}.initial;
        
        constraintEquations = [constraintEquations
                               (jointNormal(1)*cos(bodyPosition(3,1)) - jointNormal(2)*sin(bodyPosition(3,1)))*(bodyPosition(1,1) - bodyPosition(1,2) + jointLocation(1,1)*cos(bodyPosition(3,1)) - jointLocation(1,2)*cos(bodyPosition(3,2)) - jointLocation(2,1)*sin(bodyPosition(3,1)) + jointLocation(2,2)*sin(bodyPosition(3,2))) + (jointNormal(2)*cos(bodyPosition(3,1)) + jointNormal(1)*sin(bodyPosition(3,1)))*(bodyPosition(2,1) - bodyPosition(2,2) + jointLocation(2,1)*cos(bodyPosition(3,1)) - jointLocation(2,2)*cos(bodyPosition(3,2)) + jointLocation(1,1)*sin(bodyPosition(3,1)) - jointLocation(1,2)*sin(bodyPosition(3,2)))
                                                bodyPosition(3,1) - bodyPosition(3,2) - (bodyPosition0(3,1)-bodyPosition0(3,2))];
    elseif strcmp(jointType,'1DOF')
        bodyPosition0 = InputParameters.joints{n}.initial;
        
        constraintEquations = [constraintEquations
                                bodyPosition(InputParameters.joints{n}.DOF,1) - bodyPosition(InputParameters.joints{n}.DOF,2) - (bodyPosition0(InputParameters.joints{n}.DOF,1)-bodyPosition0(InputParameters.joints{n}.DOF,2))];
        
    end 
end

%% Defining Time Dependent Constraint Equations
for s = 1:numel(InputParameters.timeConstraints)
%     bodyDOF = 3*(numel(InputParameters.timeConstraints(s).body)-1)+1;
    bodyDOF = rangeCal(InputParameters.timeConstraints(s).body);
    constraintDOF = bodyDOF(InputParameters.timeConstraints(s).DOF);
    constraintExpression = InputParameters.timeConstraints(s).fun;
    
    constraintEquations = [constraintEquations; constraintExpression(time)-position(constraintDOF)];
end

end