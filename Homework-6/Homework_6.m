clear all;
clc;

syms r phi
c = 3; % in "m"
h = 4; % in "m"
r0= 4; % in "m"
phi0=pi/4; % in "rad"
eps = 1e-6; % assumed value of tolerance

f1 = @(r) r.^2-c^2-h^2;
f2 = @(phi) phi.^1-atan(h/c);
df1 = matlabFunction( diff(f1(r)) ); % first order differentiation of f1
df2 = matlabFunction( diff(f2(phi)) ); % first order differentiation of f2


r = Newton_Raphson(f1,df1,r0,eps) % in "m"
phi = Newton_Raphson(f2,df2,phi0,eps) % in "rad"

