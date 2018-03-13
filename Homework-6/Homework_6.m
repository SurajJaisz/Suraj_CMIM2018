clear all;
clc;

c = 3; % in "m"
h = 4; % in "m"
r0= 4; % in "m"
phi0=pi/4; % in "rad"

% f1 = @r r.^2-c^2-h^2=0
% f2 = @phi phi.-atan(h/c)=0
% df1 = diff(f1,r)
% df2 = diff(f2,phi)

r = Newton_Raphson(@r r.^2-c^2-h^2=0,diff(f1,r),r0,eps) 