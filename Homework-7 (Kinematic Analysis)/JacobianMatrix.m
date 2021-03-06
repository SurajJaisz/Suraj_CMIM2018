function Cq = JacobianMatrix(InputParameters,position)
%The Constraint Equations of the System
%   Detailed explanation goes here
Cq = [];

%% Defining Jacobian for Joints (only Revolute Joint)
for n = 1:numel(InputParameters.joints)
    jointType = InputParameters.joints(n).type;
    jointBodies = InputParameters.joints(n).bodies;
    jointLocation = InputParameters.joints(n).location;
    
    if jointBodies(1) == 0
        bodyPosition = [[0;0;0], position(rangeCal(jointBodies(2)))];
    elseif jointBodies(2) == 0
        bodyPosition = [position(rangeCal(jointBodies(1))), [0;0;0]];
    else
        bodyPosition = [position(rangeCal(jointBodies(1))), position(rangeCal(jointBodies(2)))];
    end
    
    if strcmp(jointType,'revolute')
        
        Cqn = ...
             [1, 0, -jointLocation(1,1)*sin(bodyPosition(3,1))-jointLocation(2,1)*cos(bodyPosition(3,1)), -1,  0,  jointLocation(1,2)*sin(bodyPosition(3,2))+jointLocation(2,2)*cos(bodyPosition(3,2))
              0, 1,  jointLocation(1,1)*cos(bodyPosition(3,1))-jointLocation(2,1)*sin(bodyPosition(3,1)),  0, -1, -jointLocation(1,2)*cos(bodyPosition(3,2))+jointLocation(2,2)*sin(bodyPosition(3,2))];
        
        Cqt = zeros(2,length(position));
        if jointBodies(1) == 0
            Cqt(:,rangeCal(jointBodies(2))) = Cqt(:,rangeCal(jointBodies(2))) + Cqn(:,4:6);
        elseif jointBodies(2) == 0
            Cqt(:,rangeCal(jointBodies(1))) = Cqt(:,rangeCal(jointBodies(1))) + Cqn(:,1:3);
        else
            Cqt(:,[rangeCal(jointBodies(1)),rangeCal(jointBodies(2))]) = Cqt(:,[rangeCal(jointBodies(1)),rangeCal(jointBodies(2))]) + Cqn;
        end
        
        Cq = [Cq; Cqt];
    end
end

%% Defining Time Dependent Constraint
% for n = 1:numel(InputParameters.timeConstraints)
%     bodyDOF = 3*(numel(InputParameters.timeConstraints(n).body)-1)+1;
%     constraintDOF = bodyDOF(InputParameters.timeConstraints(n).DOF);
%     Cqb = zeros(1,length(position));
%     Cqb(constraintDOF) = -1;
%     
%     Cq = [Cq; Cqb];
% end
for j = 1:numel(InputParameters.timeConstraints)
    bdof = rangeCal(InputParameters.timeConstraints(j).body); % 3*(numel(InputParameters.timeConstraints(n).body)-1)+1;
    cdof = bdof(InputParameters.timeConstraints(j).DOF);
    
    Cqb = zeros(1,length(position));
    Cqb(cdof) = -1;
        
    Cq = [Cq; Cqb];
end

end