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
setup

%%
clc;
clear all;
close all;
% fix random number generator
rng(3);

%% get parameters and define system and controller
params = get_params('params_PE2');
sys = LinearSystem(params.sys);
% If you have Mosek installed, we suggest passing 'mosek' as a third 
% argument in every RMPC constructor call. 
ctrl = RMPC(sys, params.ctrl);

%% Exercise 1a/1b)
% 1a) Compute an ellipsoidal RPI set in compute_tightening in the RMPC class. 
% 1b) Try various scalar values for rho and observe the produced plots. Choose a
%     value for rho making sure the resulting RPI set is contained in the
%     state constraints

rho = 0; % TODO: Try different values for rho and fix it for the remainder of the exercise
[x_tight, u_tight, P, K, delta] = ctrl.compute_tightening(rho);
X_tight = Polyhedron(sys.X.A, sys.X.b - x_tight);
U_tight = Polyhedron(sys.U.A, sys.U.b - u_tight);

figIdx = 1:4;
if params.plot.show
    plot_RMPC(figIdx(1), params.sim.nrSteps, X_tight, U_tight, ...
        sys.X, sys.U, P, delta, params.plot)
end

%% Exercise 1c
% Implement the robust MPC problem in RMPC.
% Then we simulate the closed-loop system for nrSteps time steps and nrTraj
% trajectories starting in x_0.

% Use 'mosek' as a third argument here in the RMPC construction for a
% speedup in computation.
% You may also change the number of computed trajectories, i.e.
% params.sim.nrTraj in the params_PE2.m file if you encounter long
% computation times.
ctrl = RMPC(sys, params.ctrl, rho);
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
        u(j,i) = v(1)+ctrl.K*(x(j,:,i)'-z{1});
        x(j+1,:,i) = sys.step(x(j,:,i)', u(j,i));
    end
end

%% plot results
if params.plot.show
    plot_x('state-time', figIdx(2), x, sys.X, params.plot);
    plot_x('state-state', figIdx(3), x, sys.X, params.plot);
    plot_u(figIdx(4), u, sys.U, params.plot);
end

%% Cleanup
cleanup;