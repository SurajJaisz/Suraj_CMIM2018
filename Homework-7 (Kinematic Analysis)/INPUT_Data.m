clear; clc; close all;
InputParameters = struct(); % Structure to input all the parameters

%% Defining Time Step and Run Time for Simulation
tstep = 0.01; % in seconds
tfinal = 2; % in seconds
InputParameters.tspan = 0:tstep:tfinal;

%% Defining Numerical Parameters for the System
L = 1; % in meters
omega = -1;

%% INPUT THE BODIES DETAILS HERE (Absolute Coordinate System)
% First line defines body type.
% Second line defines length of the body
% Third line defines body's global position vector initially.

type = 'bar';
length = L;
globalPosition = [0 L/2 pi/2]';
body1 = struct('type',type,'length',length,'position',globalPosition);

type = 'bar';
length = L;
globalPosition = [L/2 L 0]';
body2 = struct('type',type,'length',length,'position',globalPosition);

type = 'bar';
length = L;
globalPosition = [L L/2 -pi/2]';
body3 = struct('type',type,'length',length,'position',globalPosition);

InputParameters.bodies = [body1,body2,body3];

%% INPUT THE CONSTRAINTS HERE (JOINTS)
% First line indicates the joint type.
% Second line indicates the bodies involved in the joint (0 indicates
% ground).
% Third line indicates the joint location following body reference
% coordinate system.

jointType = 'revolute';
jointBodies = [0 1];
jointLocation = [[0 0]' [-L/2 0]'];
joint1 = struct('type',jointType,'bodies',jointBodies,'position',jointLocation);

jointType = 'revolute';
jointBodies = [1 2];
jointLocation = [[L/2 0]' [-L/2 0]'];
joint2 = struct('type',jointType,'bodies',jointBodies,'position',jointLocation);

jointType = 'revolute';
jointBodies = [2 3];
jointLocation = [[L/2 0]' [-L/2 0]'];
joint3 = struct('type',jointType,'bodies',jointBodies,'position',jointLocation);

jointType = 'revolute';
jointBodies = [3 0];
jointLocation = [[L/2 0]' [L 0]'];
joint4 = struct('type',jointType,'bodies',jointBodies,'position',jointLocation);

InputParameters.joints = [joint1,joint2,joint3,joint4];

%% INPUT TIME DEPENDENT CONSTRAINT EQUATION HERE
% A function: f(t)=omega*t is considered.
constraintBody = 1; % Input the affected body ID
constraintDOF = 3; % Input the affetced DOF of that body
constraintFun = @(t) omega*t; % Constraint function
constraintFunD = @(t) omega; % Derivative of constraint function
constraintFunDD = @(t) 0; % Second derivative of constraint function
timeConstraint1 = struct('body',constraintBody,'DOF',constraintDOF,'fun',constraintFun,'funDiff',constraintFunD,'funDDiff',constraintFunDD);

InputParameters.timeConstraints = [timeConstraint1];

%% Solving Kinematics of the system
