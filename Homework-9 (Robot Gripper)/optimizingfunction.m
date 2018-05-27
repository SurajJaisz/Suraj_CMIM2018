function fun = optimizingfunction(q0)
%One half of the robot gripper for optimizing points B and C
%   Detailed explanation goes here

%clear; clc; close all;
InputParameters = struct(); % Structure to input all the parameters

% ===============================INPUT THE SYSTEM PARAMETERS HERE===============================

%% Defining Time Step and Run Time for Simulation
InputParameters.tstep = 0.01; % in seconds
tfinal = 1; % in seconds
InputParameters.tspan = 0:InputParameters.tstep:tfinal;
InputParameters.solver = 'ode45';
InputParameters.solverOptn = odeset;

%% Defining Numerical Parameters for the System
L1 = 6; % in meters
m = 2; % in kg
% omega = -1;
F = -400;
rhoL = 2;
InputParameters.gravity = [0;0]; % Defining gravity
InputParameters.alpha = 10;
InputParameters.beta = 10;

%% INPUT POINTS A, B, C, and K
pA = [0;0];
pKu = [9;6];
pBu = q0(1:2); %[2;4]; % Need to optimize this points
pCu = q0(3:4); %[-3;4]; % Need to optimize this points

% % Mirror image of above points about X-axis
% pBl = [pBu(1); -pBu(2)];
% pCl = [pCu(1); -pCu(2)];
% pKl = [pKu(1); -pKu(2)];

% Calculating length of the links
L2a = norm(pBu-pKu);
L2b = norm(pA-pBu);
L3 = norm(pBu-pCu);

%% INPUT THE BODIES DETAILS HERE (Absolute Coordinate System, 2D System)
% First line defines body type.
% Second line defines length of the body
% fourth line defines the mass of the body
% third line defines body's global position vector initially (by endpoints).

type = 'bar';
length = L1;
globalPosition = [pA-[L1;0],pA];    % by endpoints
body1 = struct('type',type,'length',length,'position',globalPosition);
body1.mass = rhoL*norm(body1.position(:,1)-body1.position(:,2));

type = 'bar';
length = L2a;
% mass = rhoL*L2a;
globalPosition = [pBu,pKu];    % by endpoints
body2 = struct('type',type,'length',length,'position',globalPosition);
body2.mass = rhoL*norm(body2.position(:,1)-body2.position(:,2));

type = 'bar';
length = L2b;
% mass = rhoL*L2b;
globalPosition = [pA,pBu];    % by endpoints
body3 = struct('type',type,'length',length,'position',globalPosition);
body3.mass = rhoL*norm(body3.position(:,1)-body3.position(:,2));

type = 'bar';
length = L3;
% mass = rhoL*L3;
globalPosition = [pBu,pCu];    % by endpoints
body4 = struct('type',type,'length',length,'position',globalPosition);
body4.mass = rhoL*norm(body4.position(:,1)-body4.position(:,2));

% type = 'bar';
% length = L2a;
% mass = rhoL*L2a;
% globalPosition = [pBl,pKl];    % by endpoints
% body5 = struct('type',type,'length',length,'mass',mass,'position',globalPosition);
% 
% type = 'bar';
% length = L2b;
% mass = rhoL*L2b;
% globalPosition = [pA,pBl];    % by endpoints
% body6 = struct('type',type,'length',length,'mass',mass,'position',globalPosition);
% 
% type = 'bar';
% length = L3;
% mass = rhoL*L3;
% globalPosition = [pBl,pCl];    % by endpoints
% body7 = struct('type',type,'length',length,'mass',mass,'position',globalPosition);

InputParameters.bodies = [body1,body2,body3,body4];%,body5,body6,body7];

%% INPUT THE CONSTRAINTS HERE (JOINTS)
% First line indicates the joint type.
% Second line indicates the bodies involved in the joint (0 indicates
% ground).
% Third line indicates the joint location following body reference
% coordinate system.

jointType = 'translational';
jointBodies = [0, 1];
jointLocation = [pA, pA-[L1/2;0]];
normal = [0;1];          % Normal vector in 1st body reference frame
joint1 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation,'normal',normal);


jointType = 'revolute';
jointBodies = [1, 3];
jointLocation = [[L1/2;0],[-L2b/2;0]];
joint2 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);

jointType = 'revolute';
jointBodies = [3, 2];
jointLocation = [[L2b/2;0],[-L2a/2;0]];
joint3 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);

jointType = 'revolute';
jointBodies = [3, 4];
jointLocation = [[L2b/2;0],[-L3/2;0]];
joint4 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);

jointType = 'revolute';
jointBodies = [4, 0];
jointLocation = [[L3/2;0], pCu];
joint5 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);

jointType = '1DOF';
jointBodies = [2, 3];
jointDOF = 3;
joint6 = struct('type',jointType,'bodies',jointBodies,'DOF',jointDOF);

% jointType = 'revolute';
% jointBodies = [1, 6];
% jointLocation = [[L1/2;0],[-L2b/2;0]];
% joint7 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);
% 
% jointType = 'revolute';
% jointBodies = [6, 5];
% jointLocation = [[L2b/2;0],[-L2a/2;0]];
% joint8 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);
% 
% jointType = 'revolute';
% jointBodies = [6, 7];
% jointLocation = [[L2b/2;0],[-L3/2;0]];
% joint9 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);
% 
% jointType = 'revolute';
% jointBodies = [7, 0];
% jointLocation = [[L3/2;0],pCl];
% joint10 = struct('type',jointType,'bodies',jointBodies,'location',jointLocation);
% 
% jointType = '1DOF';
% jointBodies = [5, 6];
% jointDOF = 3;
% joint11 = struct('type',jointType,'bodies',jointBodies,'DOF',jointDOF);


InputParameters.joints = {joint1,joint2,joint3,joint4,joint5,joint6};%,joint7,joint8,joint9,joint10,joint11};

% % %% INPUT TIME DEPENDENT CONSTRAINT EQUATION HERE
% % % A function: f(t)=omega*t is considered.
% % constraintBody = 1; % Input the affected body ID
% % constraintDOF = 3; % Input the affetced DOF of that body
% % constraintFun = @(t) (pi/2)+omega*t; % Constraint function
% % constraintFunD = @(t) omega; % Derivative of constraint function
% % constraintFunDD = @(t) 0; % Second derivative of constraint function
% % timeConstraint1 = struct('body',constraintBody,'DOF',constraintDOF,'fun',constraintFun,'funDiff',constraintFunD,'funDDiff',constraintFunDD);
% % 
InputParameters.timeConstraints = {};

%% INPUT THE POINT FORCES
force1.body = 1;                    % Affected body
force1.forcevector = @(t) [F;0];    % Force vector (function of time)
force1.location = [0;0];            % Effect point coordinates (body relative)

InputParameters.forces = {force1};

% ===============================INPUT THE SYSTEM PARAMETERS END'S HERE===============================

%% Calculating Position and Length, if bars are defined by points.
bodies = InputParameters.bodies;
for i = 1:numel(bodies)
    if strcmp(InputParameters.bodies(i).type,'bar')
        if isfield(bodies(i),'position')
            bodypoints = bodies(i).position;
            bodies(i).positionCal = [mean(bodypoints,2); atan2((bodypoints(2,2)-bodypoints(2,1)),(bodypoints(1,2)-bodypoints(1,1)))];
            bodies(i).lengthCal = norm(bodypoints(:,1)-bodypoints(:,2));
        end
    end
end
InputParameters.bodies = bodies; 

%% Program Automatically Selecting the type of Analysis (Kinematic or Dynamic)

InputParameters.numGenCood = 3*numel(InputParameters.bodies); % calculating number of generalized coordinates
% C = constraintEquations(InputParameters,zeros(InputParameters.numGenCood,1),0);
C = Ct(InputParameters,0);
InputParameters.numConsEqn = numel(C);% calculating number of constarint equations
InputParameters.numDOF = InputParameters.numGenCood-InputParameters.numConsEqn; % calculating number of degree of freedom of the system

% Calculating Initial Position
x0 = zeros(InputParameters.numGenCood,1);
for i = 1:numel(InputParameters.bodies)
   x0(3*(i-1)+1:3*i) = InputParameters.bodies(i).positionCal;
end
InputParameters.x0 = x0; % Calculating Initial position

% Calculating Initial Value for Translational and 1 DOF Joint
joints = InputParameters.joints;
for j = 1:numel(joints)
    if strcmp(joints{j}.type,'translational') || strcmp(joints{j}.type,'1DOF')
        bodyposition0 = bodyPosition(joints{j}.bodies,x0);
        joints{j}.initial = bodyposition0;
    end
end
InputParameters.joints = joints;

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

%% Calculating fun = xK_max - xK_min

x2 = x(rangeCal(2),:);
xK = zeros(2,numel(t));
for i = 1:numel(t)
    xK(:,i) = x2(1:2,i) + [cos(x2(3,i)) -sin(x2(3,i)); sin(x2(3,i)) cos(x2(3,i))]*[L3/2;0];
    if xK(2,i) < 2
        xK(:,i:end) = [];
        break
    end
end

fun = max(xK(1,:))-min(xK(1,:));

end

