function Q = systemForceVector (InputParameters,x,t)
% Calculating total force vector of the system
%   Detailed explanation goes here

Q = zeros(3*numel(InputParameters.bodies),1);

for i = 1: numel(InputParameters.bodies)
    bodyForces = {};
    for f = 1:numel(InputParameters.forces)
        if InputParameters.forces{f}.body == i
            bodyForces = [bodyForces, InputParameters.forces{f}];
        end
    end
    
    xb = x(rangeCal(i));
    
    Q(rangeCal(i)) = Q(rangeCal(i)) + bodyForceVector(InputParameters.bodies(i),bodyForces,xb,InputParameters.gravity,t);
end

end

