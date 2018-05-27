function Qbody = bodyForceVector(body,forces,position,gravity,time)
%Function for body force vector
%   Detailed explanation goes here

m = body.mass;
type = body.type;

if strcmp(type,'bar')
    
    Qbody = [m*gravity;0]; % Force vector on a body
    
    % For Generalized Point Force
    for f=1:numel(forces)
    fVec = forces{f}.forcevector(time);
    fLoc = forces{f}.location;
    Qbody = Qbody + [fVec; fVec'*[cos(position(3)) -sin(position(3)); sin(position(3)) cos(position(3))]*fLoc];
    end
end

end

