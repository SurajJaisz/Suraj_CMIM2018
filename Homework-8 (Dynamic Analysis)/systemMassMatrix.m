function M = systemMassMatrix (InputParameters)
% Calculating total mass matrix of the system
%   Detailed explanation goes here

M = zeros(numel(InputParameters.bodies));

for i = 1: numel(InputParameters.bodies)
    
    if strcmp(InputParameters.bodies(i).type,'bar')
        m(i) = InputParameters.bodies(i).mass;
        l(i) = InputParameters.bodies(i).length;
        
        M(rangeCal(i),rangeCal(i)) = diag([m(i), m(i), m(i)*l(i)^2/12]); % Mass matrix of the bodies combined
    end
    
%     M(rangeCal(i),rangeCal(i)) = M(rangeCal(i),rangeCal(i)) + Mbody;
end

end

