clear; clc; close all;
InputParameters = struct(); % Structure to input all the parameters

% ===============================INPUT THE SYSTEM PARAMETERS HERE===============================

%% Defining Time Step and Run Time for Simulation
InputParameters.tstep = 0.01; % in seconds
tfinal = 1; % in seconds
InputParameters.tspan = 0:InputParameters.tstep:tfinal;

%% Defining Numerical Parameters for the System
L = 1; % in meters
m = 2; % in kg
% omega = -1;
omega = -1;

%% INPUT THE BODIES DETAILS HERE (Absolute Coordinate System, 2D System)
% First line defines body type.
% Second line defines length of the body
% Third line defines the mass of the body
% Fourth line defines body's global position vector initially.

type = 'bar';
length = L;
mass = m;
globalPosition = [0, L/2, pi/2]';
% globalPosition = [L/2*cos(pi/2), L/2*sin(pi/2), pi/2]';
% globalPosition = [L/2*cos(pi/4), L/2*sin(pi/4), pi/4]';
body1 = struct('type',type,'length',length,'mass',mass,'position',globalPosition);

type = 'bar';
length = L;
mass = m;
globalPosition = [L/2, L, 0]';
% globalPosition = [L*cos(pi/2)+L/2, L*sin(pi/2), 0]';
% globalPosition = [L*cos(pi/4)+L/2, L*sin(pi/4), 0]';
body2 = struct('type',type,'length',length,'mass',mass,'position',globalPosition);

type = 'bar';
length = L;
mass = m;
globalPosition = [L, L/2, -pi/2]';
% globalPosition = [3*(L/2)*cos(pi/2)+L/2, L/2*sin(pi/2), -pi/2]';
% globalPosition = [3*(L/2)*cos(pi/4)+L/2, L/2*sin(pi/4), -pi/4]';
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

%% INPUT TIME DEPENDENT CONSTRAINT EQUATION HERE
% A function: f(t)=omega*t is considered.
constraintBody = 1; % Input the affected body ID
constraintDOF = 3; % Input the affetced DOF of that body
constraintFun = @(t) (pi/2)+omega*t;%(pi/2)+omega*t; % Constraint function
constraintFunD = @(t) omega; % Derivative of constraint function
constraintFunDD = @(t) 0; % Second derivative of constraint function
timeConstraint1 = struct('body',constraintBody,'DOF',constraintDOF,'fun',constraintFun,'funDiff',constraintFunD,'funDDiff',constraintFunDD);

InputParameters.timeConstraints = [timeConstraint1];

% ===============================INPUT THE SYSTEM PARAMETERS END'S HERE===============================

%% Solving Kinematics of the system
[t,x,xd,xdd] = kinematicAnalysis(InputParameters);

%% Adams Results for Position, Velocity, Acceleration of Body-1 along X-Axis
[numPos] = xlsread('Position.xlsx');
[numVel] = xlsread('Velocity.xlsx'); 
[numAcc] = xlsread('Acceleration.xlsx');

%% Curve Plotting
figure
plot(t,x(1,:),'Linewidth',1.5)
grid on
hold on
plot(numPos(:,1),numPos(:,2),'--','Linewidth',1.5)
legend('Software''s result','Adams result')
ylabel('Position of body-1 along x-axis (m)')
xlabel('Time (s)')  
function_HighQualityPlot(gcf,'Times New Roman',12);
print('Position of body-1 along x-axis','-depsc') % For LaTex document

figure
plot(t,xd(1,:),'Linewidth',1.5)
grid on
hold on
plot(numVel(:,1),numVel(:,2),'--','Linewidth',1.5)
legend('Software''s result','Adams result')
ylabel('Velocity of body-1 along x-axis (m)')
xlabel('Time (s)')  
function_HighQualityPlot(gcf,'Times New Roman',12);
print('Velocity of body-1 along x-axis','-depsc') % For LaTex document

figure
plot(t,xdd(1,:),'Linewidth',1.5)
grid on
hold on
plot(numAcc(:,1),numAcc(:,2),'--','Linewidth',1.5)
legend('Software''s result','Adams result')
ylabel('Acceleration of body-1 along x-axis (m)')
xlabel('Time (s)')  
function_HighQualityPlot(gcf,'Times New Roman',12);
print('Acceleration of body-1 along x-axis','-depsc') % For LaTex document
