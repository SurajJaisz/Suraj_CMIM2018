%% Jacobian Matrix using Symbolic Computations
% Author:   Suraj Jaiswal,
% Date:     21.02.2018
%% Clearing the matlab workspace
%%
clear all;
clear;
close all;
clc;

%% Defining symbolic variables
%%
syms m k c L g real
syms x(t) theta(t)
assumeAlso(m > 0);
assumeAlso(k > 0);
assumeAlso(c > 0);

%% Writing Equation of Motion

%%
xp = diff(x, t);
xpp = diff(xp, t);

thetap = diff(theta, t);
thetapp = diff(thetap, t);

% Eq1 = [3*m 0; 0 m*L^2/3]*[xpp; thetapp] + [(3*k) (2*k*L); (2*k*L) (2*k*L^2)]*[x; theta] +  [(2*c) (c*L); (c*L) (c*L^2)]*[xp; thetap] == [0; 2*m*g]
Eq1 = (3*m)*xpp + (3*k)*x + (2*k*L)*theta + (2*c)*xp + (c*L)*thetap == 0
Eq2 = (m*L^2/3)*thetapp + (2*k*L)*x + (2*k*L^2)*theta + (c*L)*xp + (c*L^2)*thetap - 2*m*g== 0

symvar(Eq1)
symvar(Eq2)
% 
% tspan=[0 1];

% jaco = diff(Eq1(:,:), k)
%% Writing Jacobian Matrix

%%
% 
% v = [x; theta];
% % J1=jacobian(Eq1,x)
% % J2=jacobian(Eq1,theta(t))
% % J3=jacobian(Eq2,x(t))
% % J4=jacobian(Eq2,theta(t))
% % 
% % RequiredJacobian = [J1 J2; J3 J4]

RequiredJacobian = [3*k 2*k*L; 2*k*L 2*k*L^2]

%% Appllying Backward Euler to derived Jacobian
%
% s2 = x
% s4 = theta
% s1dot = s2;
% 
% s3dot = s4;
% 
% s2dot = ((3*k)*s1 + (2*k*L)*s3 + (2*c)*s2 + (c*L)*s4)/(3*m)
% 
% s4dot = ((2*k*L)*s1 + (2*k*L^2)*s3 + (c*L)*s2 + (c*L^2)*s4)/(m*L^2/3)


pInitial = 0.707;
pdotInitial = 0;

tspan=[0 10];
p0=[pInitial; pdotInitial];

sInitial = 0.04;
sdotInitial = 0;
s0=[sInitial; sdotInitial];

[t1,q1]=ode15s('MassSpringDamper',tspan,s0)
[t2,q2]=ode15s('MassSpringDamper',tspan,p0)
