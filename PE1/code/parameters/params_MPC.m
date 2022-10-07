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

function params = params_MPC()
%% Control Parameters
params.ctrl.name = 'nominal MPC';
params.ctrl.N = 10;
params.ctrl.Q = eye(2)*100;
params.ctrl.R = 10;

%% System Parameters
% system dimensions
n = 2; m = 1;
params.sys.n = n;
params.sys.m = m;
params.sys.dt = 0.1;

% linear system
k = 0; g = 9.81; l = 1.3; c = 0.5;
params.sys.A = [1, params.sys.dt;
                params.sys.dt*(-k+g/l), 1 - params.sys.dt*c];
params.sys.B = [0;params.sys.dt];
params.sys.C = eye(n);
params.sys.D = zeros(n,m);

% state constraints
params.sys.A_x = [1,0; -1,0; 0,1; 0,-1];
params.sys.b_x = [pi/4; pi/4; pi/3; pi/3]; %[rad]
% input constraints
params.sys.A_u = [1;-1];
params.sys.b_u = [5;5]; %[1/s^2]
% noise description
params.sys.A_w = [];
params.sys.b_w = [];
% noise distribution
params.sys.generateNoise = @zero;
params.sys.noiseArgs = {params.sys.n};

%% Simulation Parameters
params.sim.nrSteps = 30;
params.sim.nrTraj = 1;
params.sim.x_0 = [20*pi/180;10*pi/180]; %[rad]

%% Plot Parameters
params.plot.show = true;
params.plot.height = 350;
params.plot.width = 900;
params.plot.alpha = 1;
params.plot.color = [0.7216, 0.1490, 0.0039];
params.plot.lw = 1;
end