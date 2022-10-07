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

%% Setup
clc;
clear all;
close all;
% fix random number generator
rng(1);

%% get parameters and define system and controller
params = get_params('params_MPC');
sys = LinearSystem(params.sys);
ctrl = MPC(sys, params.ctrl);

%% simulate the closed-loop system for nrSteps time steps and nrTraj
%  trajectories starting in x_0
nrSteps = params.sim.nrSteps;
nrTraj = params.sim.nrTraj;
x_0 = params.sim.x_0;

% allocate state and input trajectories
x = zeros(nrSteps+1,size(x_0,1),nrTraj);
u = zeros(nrSteps,nrTraj);
x(1,:,:) = repmat(x_0,[1,1,nrTraj]);

% control-loop
for i=1:nrTraj
    for j=1:nrSteps
        [sol, ~, errmsg] = ctrl.solve(x(j,:,i)', {}, 0);
        if errmsg
            error(errmsg);
        end
        u(j,i) = sol(1);
        x(j+1,:,i) = sys.step(x(j,:,i)', u(j,i));
    end
end

%% plot results
figIdx = 1:3;
if params.plot.show
    plot_x('state-time', figIdx(1), x, sys.X, params.plot);    
    plot_x('state-state', figIdx(2), x, sys.X, params.plot);
    plot_u(figIdx(3), u, sys.U, params.plot);
end