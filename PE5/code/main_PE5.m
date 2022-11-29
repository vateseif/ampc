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

%% reset everything and configure path
clear all;
close all;
clc;

% fix random number generator
rng(10);

figIdx = 1:7;

%% get parameters and define system and controller
params = get_params('params_PE5');
sys = LinearAffineSystem(params.sys);
ctrl = SMMPC(sys, params.ctrl);

%% Exercise 1 and 2
% Implement the set-membership update for polytopic parameter and
% disturbance sets and perform the update with the given data
%%% TODO: - implement the update_estimate method in SM.m
%         - implement the remove_redundant_halfspace in SM.m
param_est = SM(sys);

% Given Data
x_prev = [1;2];
u_prev = 3;
x_new = [1.3; -0.18];

Omega_0 = param_est.Omega;
Delta = param_est.update_estimate(x_new, x_prev, u_prev);
Omega_1 = param_est.Omega;

plot_PE5(figIdx(1), Omega_0, Delta, Omega_1, params.plot)

%% Exercise 3: Robust constraint tightening
% TODO: Implement the compute_robust_tightening method in SMMPC.m

param_est = SM(sys);
rho = 0.95; lambda = 0.95; % don't modify these values
[K,P] = ctrl.compute_controller(sys, rho, lambda);

% compute disturbance reachable sets and corresponding tightenings
W_theta = param_est.estimate_W_theta();
F = ctrl.compute_robust_tightening(K, sys.W + W_theta);

% compute tightened state constraints
X_tight{1} = sys.X;
for i=2:length(F)
    X_tight{i} = sys.X - F{i};
end
% compute tightened input constraints
U_tight{1} = sys.U;
for i=2:length(F)
    U_tight{i} = sys.U - K*F{i};
end

% plot tightenings
plot_rec10(figIdx(2), params.sim.nrSteps, X_tight, U_tight, ...
           sys.X, sys.U, F, params.plot)

%% Exercise 4: Set Membership MPC
% TODO: Implement the SMMPC problem in the constructor of SMMPC.m

% then, simulate the closed-loop system for nrSteps time steps and nrTraj
% reset system, controller and parameter estimator
sys = LinearAffineSystem(params.sys);
sys_true = LinearSystem(params.sys_true);
ctrl = SMMPC(sys, params.ctrl);
param_est = SM(sys);

%  trajectories starting in x_0
nrSteps = params.sim.nrSteps;
nrTraj = params.sim.nrTraj;
x_0 = params.sim.x_0;

% allocate state and input trajectories
x = zeros(nrSteps+1,size(x_0,1),nrTraj); 
u = zeros(nrSteps,nrTraj);
x(1,:,:) = repmat(x_0,[1,1,nrTraj]);

% allocate learned parameter history
alpha = zeros(nrSteps,nrTraj);
beta = zeros(nrSteps,nrTraj);

% control-loop
for i=1:nrTraj
    for j=1:nrSteps
        W_theta = param_est.estimate_W_theta();
        [~, hx_tight, hu_tight] = ctrl.compute_robust_tightening(K, sys.W + W_theta);
        % compute A_\hat{\theta}_k
        A_theta = sys.A_delta{1};
        for k=2:length(sys.A_delta)
            A_theta = A_theta + sys.A_delta{k}*param_est.theta_hat(k-1);
        end
        
        [sol, errmsg] = ctrl.solve(x(j,:,i)', {A_theta, hx_tight, hu_tight}, 0);
        if errmsg
            error(errmsg);
        end
        
        u(j,i) = sol;
        x(j+1,:,i) = sys_true.step(x(j,:,i)', u(j,i));
        
        alpha(j,i) = param_est.theta_hat(1);
        beta(j,i) = param_est.theta_hat(2);

        param_est.update_estimate(x(j+1,:,i)', x(j,:,i)', u(j,i));
    end
end

% plot trajectories
plot_x('state-time', figIdx(3), x, sys.X, params.plot);
plot_x('state-state', figIdx(4), x, sys.X, params.plot);
plot_u(figIdx(5), u, sys.U, params.plot);

% plot parameter learning
figure(figIdx(6));
subplot(1,2,1);
plot(alpha,'color',[params.plot.color,params.plot.alpha],'linewidth',params.plot.lw);
hold on; plot([0, nrSteps+2], [params.sys_true.k,params.sys_true.k], 'k--', 'linewidth', 1.);
xlabel('time'); ylabel('k'); xlim([0,nrSteps+2]); ylim([min(min(alpha),params.sys_true.k)-0.1, max(max(alpha),params.sys_true.k)+0.1]); grid();
title('Learn k using SM');
subplot(1,2,2);
plot(beta,'color',[params.plot.color,params.plot.alpha],'linewidth',params.plot.lw);
hold on; plot([0, nrSteps+2], [params.sys_true.c,params.sys_true.c], 'k--', 'linewidth', 1.);
xlabel('time'); ylabel('c'); xlim([0,nrSteps+2]); ylim([min(min(beta),params.sys_true.c)-0.1, max(max(beta),params.sys_true.c)+0.1]); grid();
title('Learn c using SM');
set(gcf,'position',[100,100,params.plot.width,params.plot.height],'color','white')

%% Compute the DRS again: Let's see if the DRS shrink
F = ctrl.compute_robust_tightening(K, sys.W + W_theta);

% compute tightened state constraints
X_tight{1} = sys.X;
for i=2:length(F)
    X_tight{i} = sys.X - F{i};
end
% compute tightened input constraints
U_tight{1} = sys.U;
for i=2:length(F)
    U_tight{i} = sys.U - K*F{i};
end

% plot tightenings
plot_rec10(figIdx(7), params.sim.nrSteps, X_tight, U_tight, ...
           sys.X, sys.U, F, params.plot)


%% Simulate again with no learning: See if cost improves
% reset system, controller and parameter estimator
sys = LinearAffineSystem(params.sys);
sys_true = LinearSystem(params.sys_true);
ctrl = SMMPC(sys, params.ctrl);
param_est = SM(sys);

%  trajectories starting in x_0
nrSteps = params.sim.nrSteps;
nrTraj = params.sim.nrTraj;
x_0 = params.sim.x_0;

% allocate state and input trajectories
xx = zeros(nrSteps+1,size(x_0,1),nrTraj); 
uu = zeros(nrSteps,nrTraj);
xx(1,:,:) = repmat(x_0,[1,1,nrTraj]);

% only need to compute this once, since we don't update the estimate
W_theta = param_est.estimate_W_theta();
[~, hx_tight, hu_tight] = ctrl.compute_robust_tightening(K, sys.W + W_theta);

% control-loop
for i=1:nrTraj
    for j=1:nrSteps
        [sol, errmsg] = ctrl.solve(xx(j,:,i)', {sys.A, hx_tight, hu_tight}, 0); % use A_\bar{\theta}
        if errmsg
            error(errmsg);
        end
        
        uu(j,i) = sol;
        xx(j+1,:,i) = sys_true.step(xx(j,:,i)', uu(j,i));
    end
end

% print costs
C_SM = sum(diag(x*params.ctrl.Q*x')) + sum(diag(u*params.ctrl.R*u'));
disp(['Closed-loop cost SM-MPC: ', num2str(C_SM)]);
C_R = sum(diag(xx*params.ctrl.Q*xx')) + sum(diag(uu*params.ctrl.R*uu'));
disp(['Closed-loop cost RMPC: ', num2str(C_R)]);
disp(['Cost improvement [%]: ', num2str((C_R - C_SM)*100/C_R)]);