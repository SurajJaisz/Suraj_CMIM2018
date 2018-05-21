%% Numerical Integration and Modal Analysis Script
% Author:   Suraj Jaiswal,
% Date:     13.02.2018
% Updated:  21.05.2018

%% Clearing the matlab workspace
clear; close all; clc;

%% Defining Solver Choice and Time Settings
solver = 'odeRK4'; % Options are: 'odeRK4', 'odeSIE', 'ode45', 'ode15s'
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

if strcmp(solver,'ode15s')
    [t,y] = ode15s(fun,tspan,y0);
    [tp,p] = ode15s(funModal,tspan,p0);
    y = y';
    p = p';
elseif strcmp(solver,'ode45')   
    [t,y] = ode45(fun,tspan,y0);
    [tp,p] = ode45(funModal,tspan,p0);
    y = y';
    p = p';
elseif strcmp(solver,'odeSIE')
    u0 = y0(1:3);
    v0 = y0(4:6);
    up0 = p0(1:3);
    vp0 = p0(4:6);
    funF = @(t,v) v;
    funG = @(t,u) -(M\K)*u;
    funFmodal = @(t,vp) vp;
    funGmodal = @(t,up) -(Mm\Km)*up;
    
    [t,u,v] = odeSIE(funF,funG,tspan,u0,v0);
    [tp,up,vp] = odeSIE(funFmodal,funGmodal,tspan,up0,vp0);
    y = [u;v];
    p = [up;vp];
elseif strcmp(solver,'odeRK4')
    [t,y] = odeRK4(fun,tspan,y0);
    [tp,p] = odeRK4(funModal,tspan,p0);
end

%% Transforming back from modal coordinates
yModal = zeros(size(p));
for i = 1:length(tp)
    yModal(:,i) = [U zeros(size(U)); zeros(size(U)) U] * p(:,i);
end

%% Plotting for the third block
figure
% subplot(2,1,1)
% plot((t),y(3,:),'-','LineWidth',1.5);
% grid on
% hold on
% plot(tp,yModal(3,:),':','LineWidth',1.5);
% ylabel('Displacement (m)')
% xlabel('Time (s)')
% function_HighQualityPlot(gcf,'Times New Roman',12);
% legend('Computation using generalized coodinates','Computation using modal coordinates')

% subplot(2,1,2)
plot(t,y(3,:)-yModal(3,:),'k','Linewidth',1.5)
grid on
ylabel('Error in displacement (m)')
xlabel('Time (s)')  
function_HighQualityPlot(gcf,'Times New Roman',12);

print('Generalized and Modal Coordinates Using odeRK4','-depsc') % For LaTex document
