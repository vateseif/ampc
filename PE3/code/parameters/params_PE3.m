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

function params = params_RNMPC()
%% Control Parameters
params.ctrl.name = 'nonlinear RMPC';
params.ctrl.N = 15;
params.ctrl.Q = eye(2)*10;
params.ctrl.R = 100;

% state dependent disturbance function
dt = 0.1; mu_bar = 0.5;
params.ctrl.mu_bar = mu_bar;
params.ctrl.G = [0, 0;
                 0, dt*mu_bar];

%% System Parameters
% system dimensions
n = 2; m = 1;
params.sys.n = n;
params.sys.m = m;
params.sys.dt = dt;

% nonlinear system dynamics
k = 4; c = 1.5; g = 9.81; l = 1.3;
params.sys.f = @(x,u) [x(1) + params.sys.dt*x(2);
                       x(2) + params.sys.dt*(-k*x(1) - c*x(2) + sin(x(1))*g/l + u)] ...
                       + rand()*params.ctrl.G*x; % add state dependent disturbance
params.sys.h = @(x,u) x;

% differential dynamics
params.diff_sys.A = {[1,params.sys.dt;
                      params.sys.dt*(-k + g/l*cos(0)), 1 - params.sys.dt*c],
                     [1,params.sys.dt;
                      params.sys.dt*(-k + g/l*cos(pi/6)), 1 - params.sys.dt*c]};
params.diff_sys.B = {[0;params.sys.dt]};

% state constraints
params.sys.A_x = [1,0; -1,0; 0,1; 0,-1];
params.sys.b_x = [30*pi/180; 30*pi/180; pi/4; pi/4]; %[rad]
% input constraints
params.sys.A_u = [1;-1];
params.sys.b_u = [5;5]; %[1/s^2]
% noise description
params.sys.A_w = [];
params.sys.b_w = []; %[rad, rad, rad/s, rad/s]
% noise distribution
params.sys.generateNoise = @zero;
params.sys.noiseArgs = {params.sys.n};


%% Simulation Parameters
params.sim.nrSteps = 30;
params.sim.nrTraj = 40;
params.sim.x_0 = [20*pi/180; 10*pi/180]; %[rad]

%% Plot Parameters
params.plot.show = true;
params.plot.height = 350;
params.plot.width = 900;
params.plot.alpha = 0.2;
params.plot.color = [0.7216, 0.1490, 0.0039];
params.plot.lw = 1;
end