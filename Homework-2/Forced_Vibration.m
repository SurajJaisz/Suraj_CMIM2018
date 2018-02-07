clear all; clc
%% Forced Vibration


pInitial = 0.04;
pdotInitial = 0;

tspan=[0 7];
p0=[pInitial; pdotInitial];

tic
[t1,y1]=ode45('Forced_MassSpringDamper',tspan,p0);
toc


plot(t1,y1(:,1));
grid on
xlabel('Time (s)')
ylabel('Displacement (m)')
title('Response of a forced-damped system')