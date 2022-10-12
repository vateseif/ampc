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

function params = params_PE2()
%% Control Parameters
params.ctrl.name = 'robust MPC';
params.ctrl.N = 10;
params.ctrl.Q = 100*eye(2);
params.ctrl.R = 10;

%% Estimated System Parameters
params.sys.linear = true; %[bool]
% system dimensions
n = 2; m = 1;
params.sys.n = n;
params.sys.m = m;
params.sys.dt = 0.1;

% linear system
k = 4; g = 9.81; l = 1.3; c = 1.5;
params.sys.A = [1, params.sys.dt;
                params.sys.dt*(-k+g/l), 1 - params.sys.dt*c];
params.sys.B = [0;params.sys.dt];
params.sys.C = eye(n);
params.sys.D = zeros(n,m);

% state constraints
params.sys.A_x = [1,0; -1,0; 0,1; 0,-1];
params.sys.b_x = [30*pi/180; 30*pi/180; pi/4; pi/4]; %[rad]
% input constraints
params.sys.A_u = [1;-1];
params.sys.b_u = [5;5]; %[1/s^2]
% noise description
params.sys.A_w = [1,0; -1,0; 0,1; 0,-1];
params.sys.b_w = [0.4*pi/180; 0.4*pi/180; 0.5*pi/180; 0.5*pi/180]; %[rad, rad, rad/s, rad/s]
% noise distribution
params.sys.generateNoise = @polytope_vertices;
params.sys.noiseArgs = {Polyhedron(params.sys.A_w, params.sys.b_w).V};
% parameter description
params.sys.paramVariance = 0;
params.sys.p = 0; % Number of estimated parameters

%% Simulation Parameters
params.sim.nrSteps = 30;
params.sim.nrTraj = 20;
params.sim.x_0 = [25*pi/180;20*pi/180]; %[rad]

%% Plot Parameters
params.plot.show = true;
params.plot.height = 350;
params.plot.width = 900;
params.plot.alpha = 1;
params.plot.color = [0.7216, 0.1490, 0.0039];
params.plot.lw = 1;
end
