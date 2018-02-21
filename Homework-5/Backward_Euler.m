function [x,y]=Backward_Euler(f,xinit,yinit,xfinal,n)
% Euler approximation for ODE initial value problem
% Euler backward method
% Calculation of h from xinit, xfinal, and n
h=(xfinal-xinit)/n;
% Initialization of x and y as column vectors
x=[xinit zeros(1,n)];
y=[yinit zeros(1,n)];
% Calculation of x and y
for i=1:n
    x(i+1)=x(i)+h;
    ynew=y(i)+h*(f(x(i),y(i)));
    y(i+1)=y(i)+h*f(x(i+1),ynew);
end
end