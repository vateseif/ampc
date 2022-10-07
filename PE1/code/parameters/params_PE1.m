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

function params = params_PE1()
%%% TODO %%%
% Define unspecified parameters as laid out in the exercise

% --- start inserting here ---

%% Control Parameters
params.ctrl.name = 'nominal non-linear MPC';
params.ctrl.N = 10;
params.ctrl.Q = eye(2) * 100;
params.ctrl.R = 10;

%% System Parameters
% system dimensions
n = 2; m = 1; dt = 0.1;
params.sys.n = n;
params.sys.m = m;
params.sys.dt = dt;

% nonlinear system dynamics
k = 4; c = 1.5; l = 1.3; g = 9.81;
params.sys.f = @(x,u) [x(1)+dt*x(2); x(2)+dt*(-k*x(1)-c*x(2)+g/l*sin(x(1))+u)];
params.sys.h = @(x,u) x;

% state constraints
params.sys.A_x = [eye(2); -eye(2)];
params.sys.b_x = [deg2rad(45), deg2rad(60), deg2rad(45), deg2rad(60)]'; %[rad]
% input constraints
params.sys.A_u = [1; -1];
params.sys.b_u = [5; 5]; %[1/s^2]

% --- stop inserting here ---
%%%

% noise description
params.sys.A_w = [1,0; -1,0; 0,1; 0,-1];
params.sys.b_w = [1e-6; 1e-6; 1e-6; 1e-6];
% noise distribution
params.sys.generateNoise = @uniform;
params.sys.noiseArgs = {params.sys.A_w, params.sys.b_w};

%% Simulation Parameters
params.sim.nrSteps = 30;
params.sim.nrTraj = 1;
params.sim.x_0 = [20*pi/180;0]; %[rad]

%% Plot Parameters
params.plot.show = true;
params.plot.height = 350;
params.plot.width = 900;
params.plot.alpha = 1;
params.plot.color = [0.7216, 0.1490, 0.0039];
params.plot.lw = 1;
end