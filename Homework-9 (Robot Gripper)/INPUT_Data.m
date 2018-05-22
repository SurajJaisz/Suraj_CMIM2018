clear; clc; close all;
InputParameters = struct(); % Structure to input all the parameters

% ===============================INPUT THE SYSTEM PARAMETERS HERE===============================

%% Defining Time Step and Run Time for Simulation
tstep = 0.01; % in seconds
tfinal = 1; % in seconds
InputParameters.tspan = 0:tstep:tfinal;
InputParameters.solver = 'ode45';
InputParameters.solverOptn = odeset;

%% Defining Numerical Parameters for the System
L = 1; % in meters
m = 2; % in kg
omega = -1;
InputParameters.gravity = [0;-9.81]; % Defining gravity
InputParameters.alpha = 10;
InputParameters.beta = 10;

%% INPUT THE BODIES DETAILS HERE (Absolute Coordinate System, 2D System)
% First line defines body type.
% Second line defines length of the body
% Third line defines the mass of the body
% Fourth line defines body's global position vector initially.

type = 'bar';
length = L;
mass = m;
% globalPosition = [0, L/2, pi/2]';
globalPosition = [L/2*cos(pi/2), L/2*sin(pi/2), pi/2]';
body1 = struct('type',type,'length',length,'mass',mass,'position',globalPosition);

type = 'bar';
length = L;
mass = m;
% globalPosition = [L/2, L, 0]';
globalPosition = [L*cos(pi/2)+L/2, L*sin(pi/2), 0]';
body2 = struct('type',type,'length',length,'mass',mass,'position',globalPosition);

type = 'bar';
length = L;
mass = m;
% globalPosition = [L, L/2, -pi/2]';
globalPosition = [3*(L/2)*cos(pi/2)+L/2, L/2*sin(pi/2), -pi/2]';
body3 = struct('type',type,'length',length,'mass',mass,'position',globalPosition);

InputParameters.bodies = [body1,body2,body3];

%% INPUT THE CONSTRAINTS HERE (JOINTS)
% First line indicates the joint type.
% Second line indicates the bodies involved in the joint (0 indicates
% ground).
% Third line indicates the joint location following body reference
% coordinate system.

jointType = 'revolute';
jointBodies = [0, 1];
jointLocation = [[0;0], [-L/2;0]];
joint1 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);

jointType = 'revolute';
jointBodies = [1, 2];
jointLocation = [[L/2;0], [-L/2;0]];
joint2 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);

jointType = 'revolute';
jointBodies = [2, 3];
jointLocation = [[L/2;0], [-L/2;0]];
joint3 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);

jointType = 'revolute';
jointBodies = [3, 0];
jointLocation = [[L/2;0], [L;0]];
joint4 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);

InputParameters.joints = [joint1,joint2,joint3,joint4];

% % %% INPUT TIME DEPENDENT CONSTRAINT EQUATION HERE
% % % A function: f(t)=omega*t is considered.
% % constraintBody = 1; % Input the affected body ID
% % constraintDOF = 3; % Input the affetced DOF of that body
% % constraintFun = @(t) (pi/2)+omega*t; % Constraint function
% % constraintFunD = @(t) omega; % Derivative of constraint function
% % constraintFunDD = @(t) 0; % Second derivative of constraint function
% % timeConstraint1 = struct('body',constraintBody,'DOF',constraintDOF,'fun',constraintFun,'funDiff',constraintFunD,'funDDiff',constraintFunDD);
% % 
InputParameters.timeConstraints = [];

% ===============================INPUT THE SYSTEM PARAMETERS END'S HERE===============================

%% Program Automatically Selecting the type of Analysis (Kinematic or Dynamic)
InputParameters.numGenCood = 3*numel(InputParameters.bodies); % calculating number of generalized coordinates
C = constraintEquations(InputParameters,zeros(InputParameters.numGenCood,1),0);
InputParameters.numConsEqn = size(C);% calculating number of constarint equations
InputParameters.numDOF = InputParameters.numGenCood-InputParameters.numConsEqn; % calculating number of degree of freedom of the system

if InputParameters.numDOF > 0
    [t,x,xd,xdd] = dynamicAnalysis(InputParameters); % Solving Dynamics of the system
elseif InputParameters.numDOF == 0
    [t,x,xd,xdd] = kinematicAnalysis(InputParameters); % Solving Kinematics of the system
else
    disp('The System is Overconstarined')
    t = 0;
    x = 0;
    xd = 0;
    xdd = 0;
end

%% Visualize results
% pproc_animate(x,t,InputParameters);

%% Testing
% 
% for i = 1:numel(t)
%     Rc(:,i) = sum(abs(constraintEquations(InputParameters,x(:,i),t(i))));
% end
% % figure
% semilogy(t,abs(Rc),'k-.','LineWidth',1);
% grid on


figure
plot(t,xdd(1,:))
title('position')
% 
% figure
% plot(t,xd(1,:))
% title('velocity')
% 
% figure
% plot(t,xdd(1,:))
% title('acceleration')
