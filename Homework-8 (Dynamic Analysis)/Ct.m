function Ct = Ct(InputParameters,time)
% function Ct = Ct(InputParameters,position,velocity,time)
%Ct of the System
%   Detailed explanation goes here
Ct = [];

%% 
for n = 1:numel(InputParameters.joints)
    jointType = InputParameters.joints(n).type;
%     jointBodies = InputParameters.joints(n).bodies;
%     jointLocation = InputParameters.joints(n).location;
%     
%     if jointBodies(1) == 0
%         bodyPosition = [[0 0 0]' position(3*(numel(InputParameters.bodies(2))-1)+1:3*numel(InputParameters.bodies(2)))];
%         bodyVelocity = [[0 0 0]' velocity(3*(numel(InputParameters.bodies(2))-1)+1:3*numel(InputParameters.bodies(2)))];
%     elseif jointBodies(2) == 0
%         bodyPosition = [position(3*(numel(InputParameters.bodies(1))-1)+1:3*numel(InputParameters.bodies(1))) [0 0 0]'];
%         bodyVelocity = [velocity(3*(numel(InputParameters.bodies(1))-1)+1:3*numel(InputParameters.bodies(1))) [0 0 0]'];
%     else
%         bodyPosition = [position(3*(numel(InputParameters.bodies(1))-1)+1:3*numel(InputParameters.bodies(1))) position(3*(numel(InputParameters.bodies(2))-1)+1:3*numel(InputParameters.bodies(2)))];
%         bodyVelocity = [velocity(3*(numel(InputParameters.bodies(1))-1)+1:3*numel(InputParameters.bodies(1))) velocity(3*(numel(InputParameters.bodies(2))-1)+1:3*numel(InputParameters.bodies(2)))];
%     end
%     
    if strcmp(jointType,'revolute')
        Ct = [Ct; zeros(2,1)];
    end
end

%% 
for s = 1:numel(InputParameters.timeConstraints)
%     bodyDOF = 3*(numel(InputParameters.timeConstraints(s).body)-1)+1;
    bodyDOF = rangeCal(InputParameters.timeConstraints(s).body);
    constraintDOF = bodyDOF(InputParameters.timeConstraints(s).DOF);
    cDiff = InputParameters.timeConstraints(s).funDiff;
    
    Ct = [Ct; cDiff(time)];
end

end