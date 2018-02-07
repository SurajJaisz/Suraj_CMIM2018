%% Forced Vibration Steady-State Response Script
% Author:   Suraj Jaiswal,
% Date:     06.02.2018
%% Clearing the matlab workspace
%%
clear all;
clear;
close all;
clc;

%% Defining symbolic variables
%%
syms m k c F0 omega real
syms x(t)
assumeAlso(m > 0);
assumeAlso(k > 0);
assumeAlso(c > 0);
assumeAlso(F0 > 0);

%% Analyze forced vibration system with initial conditions
% $$m\ddot{x}+c\dot{x}+kx={F_0}sin(\omega t)$$
% $$x(0)=0, \dot{x}(0)=0$$
%%
xp = diff(x, t);
xpp = diff(xp, t);

Eq1 = m*xpp + c*xp + k*x == F0*sin(omega*t);

solution_forcedVibration = dsolve(Eq1, [xp(0) == 0, x(0) == 0] );
disp(solution_forcedVibration)
symvar(solution_forcedVibration)

%%
% The initial condition considered here will result in the steady state solution

%% Analytical Solution for the Amplitude of the Steady State Reponse
% $$X={F_0}/m\sqrt(((\omega_n)^2-(\omega)^2)^2+(2 \zeta \omega_n \omega)^2)$$
%%
syms omega_n real
syms zeta real
syms r real
syms X0

Eq2= X0*k/F0 == 1/sqrt((1-(omega/omega_n)^2)^2+(2*zeta*(omega/omega_n))^2);
steadystatesolution = solve(Eq2, X0);
disp(steadystatesolution)
symvar(steadystatesolution)

%% Replacing $$\omega/\omega_n$ by $$r$
%%
steadystatesolution_rewrite = subs(steadystatesolution, omega/omega_n, r);
disp(steadystatesolution_rewrite)
symvar(steadystatesolution_rewrite)

%% Rewriting the solution in this form
% $$X k /{F_0}=1/\sqrt((1-r^2)^2+(2 \zeta r)^2)$$
%%
steadystatesolution_rearrange = simplify(steadystatesolution_rewrite*k/F0);
disp(steadystatesolution_rearrange)
symvar(steadystatesolution_rearrange)

%% Ploting the Required Solution
%%
figure(1)
fplot(subs(steadystatesolution_rearrange, [zeta], [0:0.10:1]), [0, 10]);
grid on

%% Creating High Quality Plots for Publication
%%
xlim([0, 2.5]); % set y axis limit
ylim([0, 5]); % set y axis limit
xlabel('Frequency Ratio (r=\omega /\omega_n)', 'FontSize',12, 'FontName', 'Times New Roman'); % xlabel
ylabel('Amplification Ratio (X k/{F_0})', 'FontSize',12, 'FontName', 'Times New Roman'); % ylabel
title ('Amplitude Response in Steady State', 'FontSize',12, 'FontName', 'Times New Roman');%'FontType', "sans-serif") % title

%% Creating Annotations Seperately
%%
% Needs to be added

%% Printing Plot as a Vector Graphics to be used in Microsoft Word
%%
print('Z:\cmim2018\Suraj_CMIM2018\Homework-3\SteadtStatePlot','-dmeta')

