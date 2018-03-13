clear all;
clc;

c = 3; % in "m"
h = 4; % in "m"
r0= 4; % in "m"
phi0=pi/4; % in "rad"
eps = 1e-6; % assumed value of tolerance

r = Newton_Raphson(@(r) r.^2-c^2-h^2,@(r) r.*2,r0,eps) % in "m"
phi = Newton_Raphson(@(phi) phi.^1-atan(h/c),@(phi) 1,phi0,eps) % in "rad"