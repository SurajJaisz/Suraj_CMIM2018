function C_Gamma = C_Gamma(InputParameters,position,velocity,time)
%
%   Detailed explanation goes here
Cqq = [];

%% 
for n = 1:numel(InputParameters.joints)
    jointType = InputParameters.joints(n).type;
    jointBodies = InputParameters.joints(n).bodies;
    jointLocation = InputParameters.joints(n).location;
    
    if jointBodies(1) == 0
        bodyPosition = [[0 0 0]' position(3*(numel(InputParameters.bodies(2))-1)+1:3*numel(InputParameters.bodies(2)))];
        bodyVelocity = [[0 0 0]' velocity(3*(numel(InputParameters.bodies(2))-1)+1:3*numel(InputParameters.bodies(2)))];
    elseif jointBodies(2) == 0
        bodyPosition = [position(3*(numel(InputParameters.bodies(1))-1)+1:3*numel(InputParameters.bodies(1))) [0 0 0]'];
        bodyVelocity = [velocity(3*(numel(InputParameters.bodies(1))-1)+1:3*numel(InputParameters.bodies(1))) [0 0 0]'];
    else
        bodyPosition = [position(3*(numel(InputParameters.bodies(1))-1)+1:3*numel(InputParameters.bodies(1))) position(3*(numel(InputParameters.bodies(2))-1)+1:3*numel(InputParameters.bodies(2)))];
        bodyVelocity = [velocity(3*(numel(InputParameters.bodies(1))-1)+1:3*numel(InputParameters.bodies(1))) position(3*(numel(InputParameters.bodies(2))-1)+1:3*numel(InputParameters.bodies(2)))];
    end
    
    if strcmp(jointType,'revolute')
               
        Cqq = [Cqq
               (  jointLocation(2,1)*sin(bodyPosition(3,1)) - jointLocation(1,1)*cos(bodyPosition(3,1)))*bodyVelocity(3,1)^2 + (jointLocation(1,2)*cos(bodyPosition(3,2)) - jointLocation(2,2)*sin(bodyPosition(3,2)))*bodyVelocity(3,2)^2
               (- jointLocation(2,1)*cos(bodyPosition(3,1)) - jointLocation(1,1)*sin(bodyPosition(3,1)))*bodyVelocity(3,1)^2 + (jointLocation(2,2)*cos(bodyPosition(3,2)) + jointLocation(1,2)*sin(bodyPosition(3,2)))*bodyVelocity(3,2)^2];
    end
end


for s = 1:numel(InputParameters.timeConstraints)
    Cqq = [Cqq; 0];
end
 
Ctt = [];
for i = 1:numel(InputParameters.joints)
    if strcmp(jointType,'revolute')
        Ctt = [Ctt; zeros(2,1)];
    end
end

for j = 1:numel(InputParameters.timeConstraints)
    cDDiff = InputParameters.timeConstraints(j).funDDiff;
    Ctt = [Ctt; cDDiff(time)];
end

Cqt = zeros(size(Ctt));

C_Gamma = - Cqq - Cqt - Ctt;

end