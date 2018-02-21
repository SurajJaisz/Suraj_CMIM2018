function [sdot pdot] = MassSpringDamper (t, s, p)
m = 1;
c = 0.1;
k = 100;
L = 1;
g = 9.8;


sdot = [s(2); ((3*k)*s(1) + (2*k*L)*s(3) + (2*c)*s(2) + (c*L)*s(4))/(3*m)];

pdot = [p(2); ((2*k*L)*s(1) + (2*k*L^2)*p(1) + (c*L)*s(2) + (c*L^2)*p(2)- 2*m*g)/(m*L^2/3)];