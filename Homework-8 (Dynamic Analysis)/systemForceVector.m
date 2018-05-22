function Q = systemForceVector (InputParameters)
% Calculating total force vector of the system
%   Detailed explanation goes here

Q = zeros(3*numel(InputParameters.bodies),1);

for i = 1: numel(InputParameters.bodies)
    
    if strcmp(InputParameters.bodies(i).type,'bar')
        m(i) = InputParameters.bodies(i).mass;
        g = InputParameters.gravity;
        
        Qbody = [m(i)*g;0]; % Force vector on a body
    end
    
    Q(rangeCal(i),1) = Q(rangeCal(i),1) + Qbody;
end

end

