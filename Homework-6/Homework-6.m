clear all;
clc;
close all;

c = 3; % in "m"
h = 4; % in "m"
r0= 4; % in "m"
phi0=pi/4 % in "rad"

f1 = @r r.^2-c^2-h^2;
f2 = @phi phi.-atan(h/c);
df1 = diff(f1,r)
df2 = diff(f2,phi)
% 
% 
% suraj = brute_force_root_finder(@(x) x.^2-9, 0, 1000, 10000)
% r = Newton_Raphson(,@(x) x.*2,1.8,eps) 