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
rng(4);

%% get parameters and define system and controller
params = get_params('params_PE3');
sys = NonlinearSystem(params.sys);
ctrl = Nonlinear_RMPC(sys, params.ctrl, params.diff_sys);

%% Exercise 1a/1b
% 1a) Implement compute_tightening in the Nonlinear_RMPC class.
% 1b) Try various values for rho and observe the produced plots. Choose a
%     value for rho making sure the resulting RPI set is contained in the
%     state constraints
rho = 0.8; % TODO: set rho here
[c_x, c_u, P, K, delta, w_bar] = ctrl.compute_tightening(rho);
X_tight = Polyhedron(sys.X.A, sys.X.b - c_x*delta);
U_tight = Polyhedron(sys.U.A, sys.U.b - c_u*delta);

figIdx = 1:8;
if params.plot.show
    plot_RNMPC(figIdx(1), params.sim.nrSteps, X_tight, U_tight, ...
        sys.X, sys.U, P, delta, params.plot)
end

%% Exercise 1c
% Implement the nonlinear robust MPC problem in Nonlinear_RMPC.
% Then we simulate the closed-loop system for nrSteps time steps and nrTraj
% trajectories starting in x_0.

ctrl = Nonlinear_RMPC(sys, params.ctrl, params.diff_sys, rho);
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
        [v, z, errmsg] = ctrl.solve(x(j,:,i)', {}, 0);
        if errmsg
            error(errmsg);
        end
        u(j,i) = v(1) + ctrl.K*(x(j,:,i)'-z(:,1));
        x(j+1,:,i) = sys.step(x(j,:,i)', u(j,i));
    end
end

%% plot results
if params.plot.show
    plot_x('state-time', figIdx(2), x, sys.X, params.plot);
    plot_x('state-state', figIdx(3), x, sys.X, params.plot);
    plot_u(figIdx(4), u, sys.U, params.plot);
end
%% Exercise 2a/2b
% 2a) Implement the state dependet disturbance avoidance constraint in
%     Nonlinear_RMPC.m. Also see instructions in that file.
% 2b) Choose w_hat as 50% of the previously used bound w_bar and simulate
%     the system. Observe what happens when you change rho and w_hat.
%     Note: Too small values of w_hat will lead to feasibility issues. The
%           same holds for too small and too large values of rho.
w_hat = w_bar/2; % TODO: set w_hat here
delta = w_hat/(1-rho);
X_tight = Polyhedron(sys.X.A, sys.X.b - c_x*delta);
U_tight = Polyhedron(sys.U.A, sys.U.b - c_u*delta);

if params.plot.show
    plot_RNMPC(figIdx(5), params.sim.nrSteps, X_tight, U_tight, ...
        sys.X, sys.U, P, delta, params.plot)
end

%% Let's run the simulation again
ctrl = Nonlinear_RMPC(sys, params.ctrl, params.diff_sys, rho, w_hat);
nrSteps = params.sim.nrSteps;
nrTraj = params.sim.nrTraj;
x_0 = params.sim.x_0;

% allocate state and input trajectories
xx = zeros(nrSteps+1,size(x_0,1),nrTraj);
uu = zeros(nrSteps,nrTraj);
xx(1,:,:) = repmat(x_0,[1,1,nrTraj]);

% control-loop
for i=1:nrTraj
    for j=1:nrSteps
        [v, z, errmsg] = ctrl.solve(xx(j,:,i)', {}, 0);
        if errmsg
            error(errmsg);
        end
        uu(j,i) = v(1) + ctrl.K*(xx(j,:,i)'-z(:,1));
        xx(j+1,:,i) = sys.step(xx(j,:,i)', uu(j,i));
    end
end

%% plot results
% In orange will be the trajectories generated with the standard nonlinear
% RMPC; in blue will be the trajectories generated with the state dependent
% disturbance nonlinear RMPC
if params.plot.show
    params.plot.color = [0.7216, 0.1490, 0.0039]; % orange
    plot_x('state-time', figIdx(6), x, sys.X, params.plot);
    plot_x('state-state', figIdx(7), x, sys.X, params.plot);
    plot_u(figIdx(8), u, sys.U, params.plot);
    
    % add new trajectories in blue
    params.plot.color = [0.392, 0.584, 0.929]; % blue
    figure(figIdx(6)); hold on; plot_x('state-time', figIdx(6), xx, sys.X, params.plot);
    figure(figIdx(7)); hold on; plot_x('state-state', figIdx(7), xx, sys.X, params.plot);
    figure(figIdx(8)); hold on; plot_u(figIdx(8), uu, sys.U, params.plot);
end
