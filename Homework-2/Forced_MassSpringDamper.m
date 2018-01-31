function pdot = Forced_MassSpringDamper (t, p)
m = 1;
e = 0.1;
k = 100;
A = 5;
w_n = sqrt(k/m); % Natural frequency in "rad/s"
w_d = w_n*sqrt(1-e^2); % Damped natural frequency in "rad/s"
Td = (2*pi)/w_d; % Damped time-period in "seconds"
w = w_d;

pdot = [p(2); (((A/m)*sin(w*t))-(2*e*sqrt(k/m)*p(2))-((k/m)*p(1)))];