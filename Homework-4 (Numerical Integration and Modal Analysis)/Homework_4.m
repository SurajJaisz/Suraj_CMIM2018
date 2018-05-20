%% Numerical Integration and Modal Analysis Script
% Author:   Suraj Jaiswal,
% Date:     13.02.2018
% Updated:  21.05.2018

%% Clearing the matlab workspace
clear; close all; clc;

%% Defining Solver Choice and Time Settings
solver = 'ode15s'; % Other options are 'odeRK4', 'odeSIE', 'ode45'
tstep = 0.001;
tspan = 0:0.001:1;

%% Defining the exemplary data for the model
m1 = 2;     % mass for block-1 in Kg
m2 = 1.5;   % mass for block-2 in Kg
m3 = 1;     % mass for block-3 in Kg
k1 = 0;%20000; % stiffness for spring-1 in N/m
k2 = 15000; % stiffness for spring-1 in N/m
k3 = 10000; % stiffness for spring-1 in N/m

%% Defining mass and stiffness matrix
M = [m1     0      0;
     0      m2     0;
     0      0      m3];

K = [k1+k2     -k2       0;
      -k2      k2+k3    -k3;
       0       -k3       k3];

%% Calculating numerically the natural frequencies of the system
[U,D] = eig(K,M);
omega = sqrt(D); % Natural frequencies in rad/s
disp('The natural frequencies (rad/s) are:')
disp(omega)

%% Transforming generalized coordinates to modal coordinates
Mm = U'*M*U;
Km = U'*K*U;
I = eye(3);

if Mm == I
    disp('Modal mass matrix is as expected')
end
if Km == D
    disp('Stiffness matrix is as expected')
end

%% Non-zero initial conditions for the system
y0 = [0,0,0.1,0,0,0]'; % Starting positions and velocities
p0 = [U zeros(size(U)); zeros(size(U)) U] \ y0; % Initial conditions in modal coordinates

%% Formulating function and solving according to solver choice
fun = @(t,y)[y(4:6); -(M\K)*y(1:3)];
funModal = @(t,p)[p(4:6); -(Mm\Km)*p(1:3)];

if strcmp(solver,'ode')
