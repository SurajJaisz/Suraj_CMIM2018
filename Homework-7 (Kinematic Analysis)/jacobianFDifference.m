function JFD = jacobianFDifference(constEqs,x,h)
%Function for Calculating finite Jacobian Difference
%   Detailed explanation goes here

JFD = zeros(length(x),length(x));

for i = 1:length(x)
    x1 = x;
    x1(i) = x1(i)+h;
    f1 = constEqs(x1);
    JFD(:,i) = (f1-constEqs(x))/h;
end

