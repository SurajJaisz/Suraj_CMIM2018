clear; clc; close all;

%% INPUT THE BODIES DETAILS HERE

% First line defines body type.
% Second line defines body's global position vector.
% Third line defines length of teh body.

body1.type = ''2;
body1.length = 1;
body1.CMcord = [2; 2];

body2.mass = 3;
body2.length = 1;
body2.CMcord = [4; 2];

bodies = [body1, body2];



% Define bodies and their initial positions
type = 'slenderRod';        % Body type
position = [0 L/2 pi/2]';   % Body initial position in global coordinates
length = L;                 % Rod length
body1 = struct('type',type,'position',position,'length',length);

type = 'slenderRod';
position = [L/2 L 0]';
length = L;
body2 = struct('type',type,'position',position,'length',length);

type = 'slenderRod';
position = [L L/2 -pi/2]';
length = L;
body3 = struct('type',type,'position',position,'length',length);

data.bodies = [body1,body2,body3];



%% INPUT THE JOINT DETAILS HERE

joint1.location = [0; 0]; % with respect to body reference coordintaes system
joint1.body1id = 0;
joint1.body2id = 1;
joint1.disbody1 = [0; 0];
joint1.disbody2 = [-0.5; 0];







no_of_bodies = 1;
L=1;
q = zeros(10);

y = q;
yest= zeros(size(y));
for n = 0:2
    yest = yest + (q.^n)./factorial(n);
end

q0 = [L/2
    0
    0
    3*L/2
    0
    0];
qp0 = zeros(size(q0));

y0 = [q0; qp0];

[T, Y] = ode45(yest, tspan, y0);
