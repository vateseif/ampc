%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2022, Alexandre Didier and Jérôme Sieber, ETH Zurich,
% {adidier,jsieber}@ethz.ch
%
% All rights reserved.
%
% This code is only made available for students taking the advanced MPC 
% class in the fall semester of 2022 (151-0371-00L) and is NOT to be 
% distributed.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params = params_PE5()
%% Control Parameters
params.ctrl.name = 'SM-MPC';
params.ctrl.N = 15;
params.ctrl.Q = eye(2)*1;
params.ctrl.R = 100;

%% Estimated System Parameters
% system paramters
n = 2; m = 1; p = 2; T_s = 0.15;
params.sys.T_s = T_s; %[s]
params.sys.p = p; % Number of estimated parameters
params.sys.n=n;
params.sys.m=m;

l = 1.3; g = 9.81;
% system matrices A = A_delta{1} + d*A_delta{2}
params.sys.A_delta{1}=[1, T_s;
                       T_s*g/l, 1];
params.sys.A_delta{2}=[0,0;-T_s, 0];
params.sys.A_delta{3}=[0,0;0,-T_s];
params.sys.B_delta{1} = [0;T_s];
params.sys.C = eye(n);
params.sys.D = zeros(n, m);

% state constraints
params.sys.A_x = [1,0; -1,0; 0,1; 0,-1];
params.sys.b_x = [40*pi/180; 40*pi/180; 40*pi/180; 40*pi/180]; %[rad]
% input constraints
params.sys.A_u = [1;-1];
params.sys.b_u = [7;7]; %[1/s^2]
% noise description
params.sys.A_w = [1,0; -1,0; 0,1; 0,-1];
params.sys.b_w = [0.15*pi/180; 0.15*pi/180; 0.15*pi/180; 0.15*pi/180]; %[rad, rad, rad/s, rad/s]
% parameter description
params.sys.A_theta = [1,0; -1,0; 0,1; 0,-1];
params.sys.b_theta = [11; -8.5; % k
                      9; -7]; % c
params.sys.theta = [10; 8.2];

params.sys.generateNoise = @uniform;
params.sys.noiseArgs = {params.sys.A_w, params.sys.b_w};


%% True System Parameters
% system paramters
params.sys_true.T_s = params.sys.T_s;
params.sys_true.n = params.sys.n;
params.sys_true.m = params.sys.m;

k = 11; c = 7;
params.sys_true.k = k;
params.sys_true.c = c;

% system matrices A = A_delta{1} + d*A_delta{2}
params.sys_true.A =[1, T_s;
                    -T_s*k + T_s*g/l, 1-T_s*c];

params.sys_true.B = [0;T_s];
params.sys_true.C = eye(n);
params.sys_true.D = zeros(n,m);

% state constraints
params.sys_true.A_x = params.sys.A_x;
params.sys_true.b_x = params.sys.b_x;
% input constraints
params.sys_true.A_u = params.sys.A_u;
params.sys_true.b_u = params.sys.b_u;
% noise description
params.sys_true.A_w = params.sys.A_w;
params.sys_true.b_w = params.sys.b_w;
params.sys_true.generateNoise = @uniform;
params.sys_true.noiseArgs = {params.sys_true.A_w, params.sys_true.b_w};

%% Simulation Parameters
params.sim.nrSteps = 40;
params.sim.nrTraj = 1;
params.sim.x_0 = [37.5*pi/180;10*pi/180]; %[rad]

%% Plot Parameters
params.plot.height = 350;
params.plot.width = 900;
params.plot.alpha = 0.8;
params.plot.color = [0.7216, 0.1490, 0.0039];
params.plot.lw = 1;
end