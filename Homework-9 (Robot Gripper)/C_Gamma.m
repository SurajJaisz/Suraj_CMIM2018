function C_Gamma = C_Gamma(InputParameters,position,velocity,time)
%
%   Detailed explanation goes here
Cqq = [];

%% 
for n = 1:numel(InputParameters.joints)
    jointType = InputParameters.joints{n}.type;
    jointBodies = InputParameters.joints{n}.bodies;
    
    if jointBodies(1) == 0
        bodyPosition = [[0 0 0]' position(rangeCal(jointBodies(2)))];
        bodyVelocity = [[0 0 0]' velocity(rangeCal(jointBodies(2)))];
    elseif jointBodies(2) == 0
        bodyPosition = [position(rangeCal(jointBodies(1))) [0 0 0]'];
        bodyVelocity = [velocity(rangeCal(jointBodies(1))) [0 0 0]'];
    else
        bodyPosition = [position(rangeCal(jointBodies(1))) position(rangeCal(jointBodies(2)))];
        bodyVelocity = [velocity(rangeCal(jointBodies(1))) velocity(rangeCal(jointBodies(2)))];
    end

    if strcmp(jointType,'revolute')
        jointLocation = InputParameters.joints{n}.location;      
        Cqq = [Cqq
               (  jointLocation(2,1)*sin(bodyPosition(3,1)) - jointLocation(1,1)*cos(bodyPosition(3,1)))*bodyVelocity(3,1)^2 + (jointLocation(1,2)*cos(bodyPosition(3,2)) - jointLocation(2,2)*sin(bodyPosition(3,2)))*bodyVelocity(3,2)^2
               (- jointLocation(2,1)*cos(bodyPosition(3,1)) - jointLocation(1,1)*sin(bodyPosition(3,1)))*bodyVelocity(3,1)^2 + (jointLocation(2,2)*cos(bodyPosition(3,2)) + jointLocation(1,2)*sin(bodyPosition(3,2)))*bodyVelocity(3,2)^2];
    elseif strcmp(jointType,'translational')
        jointLocation = InputParameters.joints{n}.location;
        jointNormal = InputParameters.joints{n}.normal;
        Cqq = [Cqq
               bodyVelocity(3,1)^2*bodyPosition(1,1)*jointNormal(2)*sin(bodyPosition(3,1)) - bodyVelocity(3,1)^2*jointNormal(1)*bodyPosition(2,1)*sin(bodyPosition(3,1)) - bodyVelocity(3,1)^2*bodyPosition(1,2)*jointNormal(2)*sin(bodyPosition(3,1)) + bodyVelocity(3,1)^2*jointNormal(1)*bodyPosition(2,2)*sin(bodyPosition(3,1)) + bodyVelocity(3,1)^2*jointLocation(1,2)*jointNormal(1)*cos(bodyPosition(3,1) - bodyPosition(3,2)) + bodyVelocity(3,2)^2*jointLocation(1,2)*jointNormal(1)*cos(bodyPosition(3,1) - bodyPosition(3,2)) + bodyVelocity(3,1)^2*jointLocation(2,2)*jointNormal(2)*cos(bodyPosition(3,1) - bodyPosition(3,2)) + bodyVelocity(3,2)^2*jointLocation(2,2)*jointNormal(2)*cos(bodyPosition(3,1) - bodyPosition(3,2)) - bodyVelocity(3,1)^2*jointLocation(1,2)*jointNormal(2)*sin(bodyPosition(3,1) - bodyPosition(3,2)) + bodyVelocity(3,1)^2*jointNormal(1)*jointLocation(2,2)*sin(bodyPosition(3,1) - bodyPosition(3,2)) - bodyVelocity(3,2)^2*jointLocation(1,2)*jointNormal(2)*sin(bodyPosition(3,1) - bodyPosition(3,2)) + bodyVelocity(3,2)^2*jointNormal(1)*jointLocation(2,2)*sin(bodyPosition(3,1) - bodyPosition(3,2)) - 2*bodyVelocity(3,1)*bodyVelocity(1,1)*jointNormal(2)*cos(bodyPosition(3,1)) + 2*bodyVelocity(3,1)*jointNormal(1)*bodyVelocity(2,1)*cos(bodyPosition(3,1)) + 2*bodyVelocity(3,1)*bodyVelocity(1,2)*jointNormal(2)*cos(bodyPosition(3,1)) - 2*bodyVelocity(3,1)*jointNormal(1)*bodyVelocity(2,2)*cos(bodyPosition(3,1)) - 2*bodyVelocity(3,1)*bodyVelocity(1,1)*jointNormal(1)*sin(bodyPosition(3,1)) + 2*bodyVelocity(3,1)*bodyVelocity(1,2)*jointNormal(1)*sin(bodyPosition(3,1)) - 2*bodyVelocity(3,1)*bodyVelocity(2,1)*jointNormal(2)*sin(bodyPosition(3,1)) + 2*bodyVelocity(3,1)*bodyVelocity(2,2)*jointNormal(2)*sin(bodyPosition(3,1)) - bodyVelocity(3,1)^2*bodyPosition(1,1)*jointNormal(1)*cos(bodyPosition(3,1)) + bodyVelocity(3,1)^2*bodyPosition(1,2)*jointNormal(1)*cos(bodyPosition(3,1)) - bodyVelocity(3,1)^2*bodyPosition(2,1)*jointNormal(2)*cos(bodyPosition(3,1)) + bodyVelocity(3,1)^2*bodyPosition(2,2)*jointNormal(2)*cos(bodyPosition(3,1)) - 2*bodyVelocity(3,1)*bodyVelocity(3,2)*jointLocation(1,2)*jointNormal(1)*cos(bodyPosition(3,1) - bodyPosition(3,2)) - 2*bodyVelocity(3,1)*bodyVelocity(3,2)*jointLocation(2,2)*jointNormal(2)*cos(bodyPosition(3,1) - bodyPosition(3,2)) + 2*bodyVelocity(3,1)*bodyVelocity(3,2)*jointLocation(1,2)*jointNormal(2)*sin(bodyPosition(3,1) - bodyPosition(3,2)) - 2*bodyVelocity(3,1)*bodyVelocity(3,2)*jointNormal(1)*jointLocation(2,2)*sin(bodyPosition(3,1) - bodyPosition(3,2))
               0];
    elseif strcmp(jointType,'1DOF')
        Cqq = [Cqq; 0];
    end
end


for s = 1:numel(InputParameters.timeConstraints)
    Cqq = [Cqq; 0];
end
 
Ctt = [];
for i = 1:numel(InputParameters.joints)
    jointType = InputParameters.joints{i}.type;
    if strcmp(jointType,'revolute')
        Ctt = [Ctt; zeros(2,1)];
    elseif strcmp(jointType,'translational')
        Ctt = [Ctt; zeros(2,1)];
    elseif strcmp(jointType,'1DOF')
        Ctt = [Ctt; 0];
    end
end

for j = 1:numel(InputParameters.timeConstraints)
    cDDiff = InputParameters.timeConstraints(j).funDDiff;
    Ctt = [Ctt; cDDiff(time)];
end

Cqt = zeros(size(Ctt));

C_Gamma = - Cqq - Cqt - Ctt;

end