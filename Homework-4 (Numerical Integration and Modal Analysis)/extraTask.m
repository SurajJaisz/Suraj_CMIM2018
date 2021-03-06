%% Numerical Integration and Modal Analysis Script
% Author:   Suraj Jaiswal,
% Date:     13.04.2018
% Updated:  21.05.2018

%% Clearing the matlab workspace
clear; clc; close all;

%% Selecting test type
Case = 2;
integrator = 'simEuler';

%% Time setting
t0 = 0;
t1 = 10;
tstep = 0.001;
tspan = t0:tstep:t1;

%% Case Studies
if Case == 1    % Population with forward Euler
    % System parameters
    r = 0.5;
    n0 = 1;
    
    % Define differential equation as function
    fun = @(t,n) r*n;

    % Compute solution using forward Euler
    [t,n_euler] = odeFE(fun,n0,tspan);

    % Compute analytical solution for comparison
    analytic_fun = @(t) n0*exp(r*t);
    n_analytic = analytic_fun(tspan);

    % Plot results
    figure
    plot(tspan,n_euler,'k-','LineWidth',1.5)
    hold on
    plot(tspan,n_analytic,'k--','LineWidth',1.5)
    hold off
    grid on
    xlabel('Time $t$','Interpreter','latex')
    ylabel('Population $n$','Interpreter','latex')
    legend('Forward Euler','Analytic solution')
    function_HighQualityPlot(gcf,'Times New Roman', 12);
    print('PopulationGrowth','-depsc') % For LaTex document
    
    % Plot error
    figure
    plot(tspan,n_euler-n_analytic,'k-','LineWidth',1.5)
    hold off
    grid on
    xlabel('Time $t$','Interpreter','latex')
    ylabel('Error $n-n_{a}$','Interpreter','latex')
    ylim([-2,0])
    title(['Time step: ',num2str(tstep)])
    function_HighQualityPlot(gcf,'Times New Roman', 12);
    print('PopulationGrowth_Error','-depsc') % For LaTex document
    
elseif Case == 2    % Mass-spring-damper, several integrators
    % System parameters
    m = 1;
    k = 100;
    A = 1;
    
    if strcmp(integrator,'fwdEuler')
        % Define differential equation as function
        fun = @(t,y) [y(2); (-k/m)*y(1)];
        y0 = [A,0];

        % Solve response using forward Euler
        [t,y] = odeFE(fun,y0,tspan);
        
    elseif strcmp(integrator,'simEuler')
        % Define differential equation as functions
        fun_f = @(t,v) v;
        fun_g = @(t,u) (-k/m)*u;
        u0 = A;
        v0 = 0;

        % Solve response using semi-implicit Euler
        [t,u,v] = odeSIE(fun_f,fun_g,tspan,u0,v0);
        y = [u;v];
        
    elseif strcmp(integrator,'rungeKutta4')
        % Define differential equation as function
        fun = @(t,y) [y(2); (-k/m)*y(1)];
        y0 = [A,0];

        % Solve response using Runge-Kutta 4th order method
        [t,y] = odeRK4(fun,tspan,y0);
        
    end
    
    %% Plot position
    figure
    plot(t,y(1,:),'k-','LineWidth',1.5)
    grid on
    ylim([-1.5,1.5])
    xlabel('Time t')
    ylabel('Position [m]')
    function_HighQualityPlot(gcf,'Times New Roman', 12);
    print('FE','-depsc') % For LaTex document
    
    % Compute and plot system energy
    T_potential = 0.5*k*y(1,:).^2;
    T_kinetic = 0.5*m*y(2,:).^2;
    T_total = T_potential+T_kinetic;
    
    figure
    plot(t,T_total,'k-','LineWidth',1.5)
    grid on
    xlabel('Time t')
    ylabel('System energy [J]')
    function_HighQualityPlot(gcf,'Times New Roman', 12);
    print('FE System Energy','-depsc') % For LaTex document
    
else
    disp('Select a case!')
end