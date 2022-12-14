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

function params = params_PE6()
%% Control Parameters
params.ctrl.name = 'stochastic LBMPC';
params.ctrl.N = 15;
params.ctrl.Q = eye(2);
params.ctrl.R = 100;

%% True System Parameters
% system paramters
params.sys_true.n=2;
params.sys_true.m=1;
params.sys_true.d = 4.9; %[1/s^2]
params.sys_true.c = 3.8; %[1/s]
params.sys_true.l = 1.3; %[m]
params.sys_true.g = 9.81; %[m/s^2]
params.sys_true.T_s = 0.1; %[s]
params.sys_true.p = 0; % Number of estimated parameters
params.sys_true.alpha_s = 0;

params.sys_true.A =[1, params.sys_true.T_s;
                    -params.sys_true.T_s*params.sys_true.d + params.sys_true.T_s*params.sys_true.g/params.sys_true.l, 1-params.sys_true.T_s*params.sys_true.c];
params.sys_true.B = [0;params.sys_true.T_s];
params.sys_true.C = eye(params.sys_true.n);
params.sys_true.D = zeros(params.sys_true.n,params.sys_true.m);

% state constraints
params.sys_true.A_x = [1,0; -1,0; 0,1; 0,-1];
params.sys_true.b_x = [pi/18;pi/6;pi/4;pi/4]; %[rad]
% input constraints
params.sys_true.A_u = [1;-1];
params.sys_true.b_u = [4;4]; %[1/s^2]

% parameter description
params.sys_true.A_theta = [];
params.sys_true.b_theta = [];
params.sys_true.paramVariance = 0;

% noise description
params.sys_true.randomlySampleW=true;
params.sys_true.A_w = [1,0; -1,0; 0,1; 0,-1];
params.sys_true.b_w = [1e-2*pi/180; 1e-2*pi/180; 1e-2*pi/180; 1e-2*pi/180]; %[rad, rad, rad/s, rad/s]
params.sys_true.generateNoise = @uniform;
params.sys_true.noiseCovariance=(0.02*pi/180)^2/12;
params.sys_true.noiseArgs = {params.sys_true.A_w, params.sys_true.b_w};

%% Nominal/Estimated System Parameters
% system paramters
params.sys.n=2;
params.sys.m=1;
params.sys.d = 5; %[N/rad]
params.sys.c = 4; %[Ns/rad]
params.sys.l = params.sys_true.l; %[m]
params.sys.g = params.sys_true.g; %[m/s^2]
params.sys.T_s = params.sys_true.T_s; %[s]
params.sys.p = 2;    % Number of estimated parameters
params.sys.alpha_s = params.sys_true.alpha_s;

% system matrices A = A_delta{1} + d*A_delta{2} + c*A_delta{3}
params.sys.A_delta{1}=[1, params.sys.T_s;
                       params.sys.T_s*params.sys.g/params.sys.l, 1];
params.sys.A_delta{2}=[0,0;-params.sys.T_s, 0];
params.sys.A_delta{3}=[0,0;0,-params.sys.T_s];
params.sys.B_delta{1} = [0;params.sys.T_s];
params.sys.C = eye(params.sys.n);
params.sys.D = zeros(params.sys.n, params.sys.m);


% state constraints
params.sys.A_x = params.sys_true.A_x;
params.sys.b_x = params.sys_true.b_x;
% input constraints
params.sys.A_u = params.sys_true.A_u;
params.sys.b_u = params.sys_true.b_u;
% noise description
params.sys.randomlySampleW=params.sys_true.randomlySampleW;
params.sys.A_w = params.sys_true.A_w;
params.sys.b_w = params.sys_true.b_w;
params.sys.generateNoise = params.sys_true.generateNoise;
params.sys.noiseCovariance=params.sys_true.noiseCovariance;
params.sys.noiseArgs = params.sys_true.noiseArgs;

% parameter set
params.sys.A_theta = [1,0; -1,0; 0,1; 0,-1];
params.sys.b_theta = [1.1*params.sys.d; -0.9*params.sys.d; params.sys.c+1; -params.sys.c+1];

params.sys.theta =[5;4];

params.sys.paramVariance = [0.06, -0.003; -0.003, 0.06];
params.sys.paramNoise = 1e-2^2*eye(2);

%% Simulation Parameters
params.sim.nrSteps = 50;
params.sim.nrTraj = 1;
params.sim.nrSim = 2000;
params.sim.x_0 = [5*pi/180; 40*pi/180]; %[rad]

%% Plot Parameters
params.plot.height = 350;
params.plot.width = 900;
params.plot.alpha = 0.02;
params.plot.color = [0.7216, 0.1490, 0.0039];
params.plot.lw = 0.5;
params.plot.title =[];
params.plot.posterior=[];
end