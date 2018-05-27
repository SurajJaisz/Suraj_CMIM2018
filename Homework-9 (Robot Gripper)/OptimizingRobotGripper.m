%% Optimizing Robot Gripper

%% Defining Initial Values for Points B and C
pBu = [2;4]; % Need to optimize this points
pCu = [-3;4]; % Need to optimize this points
q0 = [pBu; pCu];

%% Defining Upper and Lower Limits
xU = [4; 7; -1; 4];
xL = [0; 4; -5; 1];

%% Defining Optimization Options
% opts = optimoptions('fmincon', 'Algorithm', 'sqp','MaxFunctionEvaluations',1000);
% opts = optimoptions('fmincon', 'Algorithm', 'interior-point','MaxFunctionEvaluations',1000);
opts = optimoptions('fmincon', 'Algorithm', 'active-set','MaxFunctionEvaluations',1000);

%% Optimizing
[q, fval, exitflag, output] = fmincon(@optimizingfunction,q0,[],[],[],[],xL,xU,[],opts);
q
disp(output)