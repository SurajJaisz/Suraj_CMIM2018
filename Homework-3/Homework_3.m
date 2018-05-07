%% Forced Vibration Steady-State Response Script
% Author:   Suraj Jaiswal,
% Date:     06.02.2018
% Updated:  07.05.2018

%% Clearing the matlab workspace
%%
clear;
close all;
clc;

%% Defining symbolic variables
%%
syms m k c A F0 omega real
syms x(t)
assumeAlso(m > 0);
assumeAlso(k > 0);
assumeAlso(c > 0);
assumeAlso(omega > 0)
assume(t, 'real');
assumeAlso(t >= 0);
% assumeAlso(F0 > 0);

%% Analyze forced vibration system with initial conditions
% $$m\ddot{x}+c\dot{x}+kx={F_0}sin(\omega t)$$
% $$x(0)=A, \dot{x}(0)=0$$
%%
xp = diff(x, t);
xpp = diff(xp, t);

Eq1 = m*xpp + c*xp + k*x == F0*sin(omega*t);

solution_forcedVibration = dsolve(Eq1, [xp(0) == 0, x(0) == A] );
disp(solution_forcedVibration)
symvar(solution_forcedVibration)

%% Subtitution to simply the expression
%%
syms cc zeta omega_n real
assumeAlso(cc > 0)
assumeAlso(zeta > 0)
assumeAlso(omega_n > 0)

solution_forcedVibration_rewrite1 = simplify(subs(expand(solution_forcedVibration), c, cc*zeta));
solution_forcedVibration_rewrite2 = simplify(subs(expand(solution_forcedVibration_rewrite1), 4*k*m, cc*cc));
solution_forcedVibration_rewrite3 = simplify(subs(expand(solution_forcedVibration_rewrite2), cc, 2*sqrt(k*m)));
solution_forcedVibration_rewrite4 = simplify(subs(expand(solution_forcedVibration_rewrite3), sqrt(k/m), omega_n));
solution_forcedVibration_rewrite5 = simplify(subs(expand(solution_forcedVibration_rewrite4), m, k/omega_n^2));

disp(solution_forcedVibration_rewrite5)
symvar(solution_forcedVibration_rewrite5)
pretty(solution_forcedVibration_rewrite5)

%% Response from Symbolical Solution of Forced Damped Vibration 
%%
symbolicResponse_forcedVibration = simplify(expand(subs(solution_forcedVibration_rewrite5,[A,F0,k,omega,omega_n,zeta],[0,20,10,2,2,0.5])))

%% Response from Steady State Solution of Forced Damped Vibration 
%%
X = @(omega,zeta) 20/(10*sqrt((1-omega^2/2^2)^2+(2*zeta*omega/2)^2));
phi = @(omega,zeta) atan((-2*zeta*omega/2)/(1-omega^2/2^2));
analyticalResponse_forcedVibration1 = @(omega,zeta,t) X(omega,zeta)*sin(omega*t+phi(omega,zeta));
analyticalResponse_forcedVibration = @(t) analyticalResponse_forcedVibration1(2,0.5,t)

%% Function for Creating High Quality Publications Plots
%%
% function_HighQualityPlot;

%% Comparing the Symbolic Response with the Steady State Response
%%
figure
fplot(symbolicResponse_forcedVibration,[0,10],'k-','LineWidth',1.5);
hold on;
fplot(analyticalResponse_forcedVibration,[0,10],'k:','LineWidth',1.5);
hold off;
xlabel('Time');
ylabel('Position');
legend('Response from Symbolical Solution','Response from Steady State Solution');
function_HighQualityPlot(gcf,'Times New Roman',12);

%% Printing Plot as a Vector Graphics to be used in Microsoft Word
%%
print('ForcedVibrationResponse','-dmeta')

%% Amplitude Response for Forced Vibration
%%
omegaVector = 0:0.01:5;
zetaVector = [0,0.1,0.2,0.3,0.5,1];

freqRatio = omegaVector/2;
ampRatio = zeros(length(omegaVector),length(zetaVector));
phaseAngle = ampRatio;

for i = 1:length(omegaVector)
    for j = 1:length(zetaVector)
        ampRatio(i,j) = X(omegaVector(i),zetaVector(j))*10/20;
        phaseAngle(i,j) = phi(omegaVector(i),zetaVector(j));
        if phaseAngle(i,j)>0
            phaseAngle(i,j) = phaseAngle(i,j) - pi;
        end
    end
end

figure
plot(freqRatio,ampRatio,'k-','LineWidth',1);
xlim([0, 2.5]);
ylim([0, 5]);
xlabel('Frequency ratio $r=\frac{\omega}{\omega_n}$','Interpreter','latex');
ylabel('Amplification ratio $X \frac{k}{F_0}$','Interpreter','latex');

function_HighQualityPlot(gcf,'Times New Roman',12);
print('AmplitudeResponse','-dmeta')
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

