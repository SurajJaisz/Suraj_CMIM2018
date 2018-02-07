clear all; clc;

%% Analytical solution for a 1 DOF Mass-Spring-Damper System
m = 1;
% e = 0.1;
k = 100;
% A = 0;
% w_n = sqrt(k/m); % Natural frequency in "rad/s"
% w_d = w_n*sqrt(1-e^2); % Damped natural frequency in "rad/s"
% Td = (2*pi)/w_d; % Damped time-period in "seconds"
% w = w_n;



syms c real
% assumeAlso(m > 0);
% assumeAlso(k > 0);
assumeAlso(c > 0);

syms x(t)
% oms = sqrt(k/m);
syms omega real


xp = diff(x, t);
xpp = diff(xp, t);

Eq = m*xpp + c*xp + k*x;

% syms A real
solution_damped = dsolve(Eq == 0, [xp(0) == 0, x(0) == 0.04] );
disp(solution_damped)

syms cc zeta real

solution_damped_rewrite = simplify(expand(subs(subs(simplify(subs(subs(solution_damped, 4*k*m, cc*cc), c, zeta*cc)),cc,2*sqrt(k*m)),sqrt(k/m),omega)));
disp(solution_damped_rewrite)

figure
fplot(subs(solution_damped_rewrite, [omega, zeta], [10, 0.1]), [0, 1], 'g', 'LineWidth',2)
grid on
hold on

%% Numerical Solution using different ODE solvers

pInitial = 0.04;
pdotInitial = 0;

tspan=[0 1];
p0=[pInitial; pdotInitial];

tic
[t1,y1]=ode45('Forced_MassSpringDamper',tspan,p0);
toc
tic
[t2,y2]=ode15s('Forced_MassSpringDamper',tspan,p0);
toc
tic
[t3,y3]=ode113('Forced_MassSpringDamper',tspan,p0);
toc

plot(t1,y1(:,1), 'r-.');
grid on
xlabel('Time (s)')
ylabel('Displacement (m)')
title('Different Solutions for a freely-damped Spring-Damper System')
hold on;
plot(t2,y2(:,1), 'k');
plot(t3,y3(:,1), 'm--.');
legend('Exact Solution', 'ODE45 Solver', 'ODE15s Solver', 'ODE113 Solver')

%% Error estimation for ODE45 solver

% RMSE = sqrt(mean((y1 - solution_damped_rewrite).^2));  % Root Mean Squared Error
s = (y1 - solution_damped_rewrite)
plot (t1,s(:,1))