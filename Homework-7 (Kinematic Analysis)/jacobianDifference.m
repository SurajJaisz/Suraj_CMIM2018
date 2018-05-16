function [Jacobian] = jacobianDifference(constEqs,generalizedCoord,h)
%Function for Calculating Jacobian Matrix
%   Detailed explanation goes here

Jacobian = zeros(length(generalizedCoord),length(generalizedCoord));

for i = 1:(length(generalizedCoord)-1)
    generalizedCoord_New = generalizedCoord;
    generalizedCoord_New(i+1) = generalizedCoord_New(i)+h;
    functn = constEqs(generalizedCoord_New);
    Jacobian(:,i) = (functn-constEqs(generalizedCoord))/h;
end

