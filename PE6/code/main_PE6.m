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
setup
% fix random number generator
rng(11);

%% get parameters and define system
params = get_params('params_PE6');
sys_true = LinearSystem(params.sys_true);
sys = LinearAffineSystem(params.sys);

%% Compute the confidence region of parameters
% Desired probability p
p=0.5;

% Prior parameter distribution
% Mean values
d_mean = params.sys.d;
c_mean = params.sys.c;
theta_mean = [params.sys.d; params.sys.c];

% Variance
Sigma_theta = params.sys.paramVariance;

% Compute the quantile function
p_tilde=chi2inv(p,2);

%% Exercise 1: Overapproximate confidence region of parameter set
% Using the formula derived on the slides, compute a box aligned with the
% coordinate axes, which overapproximates the confidence region.

%%% TODO
% Compute the overapproximating box of the ellipsoid 
% {theta| theta'*Sigma_theta*theta <= p_tilde}, i.e. compute the
% righthandside Omega_b of the set Omega={theta| Omega_A*theta\leq Omega_b} 
% for the given Omega_A. Use the formula derived in the recitation.

% --------- Start Modifying Code Here -----------   

Omega_A = [1,0; -1,0; 0,1; 0,-1];
Omega_b = vecnorm(sqrt(p_tilde) * sqrtm(Sigma_theta) * Omega_A')';

% --------- Stop Modifying Code Here -----------

Omega = Polyhedron(Omega_A, Omega_b);

% store overapproximation in system instance
sys.Omega = Omega;

% Plot confidence region and overapproximation
figIdx = 1:12;
plot_PE6(figIdx(1), Sigma_theta, theta_mean, [sys_true.params.d; sys_true.params.c], p_tilde, Omega, params.plot)

%% Empirically test the confidence region
Ns = params.sim.nrSim;
theta_s = theta_mean + chol(Sigma_theta,'L')*randn(2,Ns);

% empirical averages
disp(['Empirical percentage of theta in the computed confidence region: ', num2str(mean(sum((theta_s-theta_mean).*(inv(Sigma_theta)*(theta_s-theta_mean)))<p_tilde))]);
disp(['Empirical percentage of theta in the polytope overapproximation of the confidence region: ', num2str(mean(all(Omega.A*(theta_s-theta_mean) <= Omega.b)))]);

% plot drawn parameters
params.plot.title='Confidence regions and parameter samples';
plot_PE6(figIdx(2), Sigma_theta, theta_mean, [sys_true.params.d; sys_true.params.c], p_tilde, Omega, params.plot)
figure(figIdx(2)); hold on;
scatter(theta_s(1,:),theta_s(2,:),'rx');

%% Exercise 2: Compute disturbance set W for lumped parametric and additive uncertainties
% Look up parameters
N = params.ctrl.N;
x_0 = params.sim.x_0;

% Look up parameterized dynamic matrices
% A(theta)=A_0+A_1*d+A_2*c
A_0 = sys.A_delta{1};
A_1 = sys.A_delta{2};
A_2 = sys.A_delta{3};

% Look up state constraint vertices
V = sys_true.X.V;

% Compute lumped disturbance set W
sigma_theta = zeros(size(V,1),1);
for i=1:size(V,1)
    sigma_theta(i) = norm(sqrt(p_tilde)*Sigma_theta^(1/2)*((A_1(2,:) + A_2(2,:)).*V(i,:))',2);
end

W_theta = Polyhedron([1, 0; -1, 0; 0, 1; 0, -1], [0; 0; max(sigma_theta); max(sigma_theta)]);
           
%%% TODO
% Compute the lumped disturbance set for the parametric uncertainty and the
% disturbance, i.e. compute the Minkowski Sum: W_theta \oplus sys_true.W

% --------- Start Modifying Code Here -----------   
sys.W = plus(W_theta, sys_true.W);

% --------- Stop Modifying Code Here -----------



% empirically test lumped disturbance set
delta_theta = theta_s - theta_mean;
w_s = [];
for i=1:Ns  
    % this computes the w_s for the given initial position
    w_s = [w_s, (A_1*delta_theta(1,i) + A_2*delta_theta(2,i))*params.sim.x_0 + sys_true.W.randomPoint()];
end

disp(['Empirical percentage of theta and w in the new disturbance set W: ', num2str(mean(all(sys.W.A*w_s <= sys.W.b)))]);

% plot
figure(figIdx(3))
p1 = sys_true.W.plot('wire', true, 'linestyle', '-', 'linewidth', 2);
hold on;
p2 = sys.W.plot('wire', true, 'linestyle', '--', 'linewidth', 2);
hold on;
p3 = scatter(w_s(1,:),w_s(2,:),'rx');
legend([p1, p2, p3], {'original W', 'lumped W', 'randomly drawn theta and w'}, 'Location', 'ne');
set(gcf,'position',[100,100,params.plot.width,2*params.plot.height],'color','white')

%% open-loop planning using robust constraint-tightening MPC
ctrl = constraint_tightening_RMPC(sys, params.ctrl);

% solve robust constraint tightening MPC for initial condition
[sol, out, errmsg] = ctrl.solve(x_0, {}, 0);
if errmsg
    error(errmsg);
end
v = out{1};
z = out{2};

% allocate state and input trajectories
x = zeros(N+1,size(x_0,1),Ns); 
u = zeros(N,Ns);
x(1,:,:) = repmat(x_0,[1,1,Ns]);

% control-loop
for i=1:Ns
    for j=1:N
        u(j,i) = ctrl.K*(x(j,:,i)' - z(:,j)) + v(j);
        x(j+1,:,i) = (A_0 + A_1*theta_s(1,i) + A_2*theta_s(2,i))*x(j,:,i)' + sys.B*u(j,i) + sys_true.W.randomPoint();
    end
end

% identify trajectories violating the constraints
constraint_violation=false(Ns,1);
for i=1:Ns
    if ~all(all(sys_true.params.A_x*squeeze(x(:,:,i))' <= sys_true.params.b_x))
        constraint_violation(i)=true;
    end
end

% plot
params.plot.color = [0.7216, 0.1490, 0.0039];
pp1=plot_x('state-time', figIdx(4), x(:,:,~constraint_violation), sys_true.X, params.plot);
p1 = plot_x('state-state', figIdx(5), x(:,:,~constraint_violation), sys_true.X, params.plot);
plot_u(figIdx(6), u, sys_true.U, params.plot);

% highlight trajectories violating the constraints
params.plot.color = [0.392, 0.584, 0.929];
params.plot.alpha = 0.4;
figure(figIdx(4)); hold on;
pp2=plot_x('state-time', figIdx(4), x(:,:,constraint_violation), sys_true.X, params.plot);

legend([pp1(1); pp2(1)], {'trajectories w/o constraint violation', 'trajectories w/ constraint violation'}, 'Location', 'southeast')

figure(figIdx(5)); hold on;
p2 = plot_x('state-state', figIdx(5), x(:,:,constraint_violation), sys_true.X, params.plot);

legend([p1(1); p2(1)], {'trajectories w/o constraint violation', 'trajectories w/ constraint violation'}, 'Location', 'southeast')

% plot parameters leading to constraint violations
params.plot.title='Confidence regions and samples which violate the state constraints using CT-MPC';
plot_PE6(figIdx(7), Sigma_theta, theta_mean, [sys_true.params.d; sys_true.params.c], p_tilde, Omega, params.plot)
figure(figIdx(7)); hold on;
scatter(theta_s(1,constraint_violation),theta_s(2,constraint_violation),'rx');

%% Simulate robust constraint-tightening MPC and simultaneously learn parameters
ctrl = constraint_tightening_RMPC(sys, params.ctrl);
nrSteps = params.sim.nrSteps;
nrTraj = params.sim.nrTraj;
x_0 = [-5*pi/180;-15*pi/180]; % new initial state to obtain enough data for estimation

% We use the same features for the Kalman Filter and for BLR
features_kf = @(x) [A_1(2,:)*x, A_2(2,:)*x];
features_blr = @(x) [A_1(2,:)*x, A_2(2,:)*x]';
data.x =[]; data.y = [];

% Exercise 3: TODO: Implement the update_estimate method for the Kalman filter in the KF.m class
% Implement the 
kf = KF(sys, theta_mean, Sigma_theta, features_kf);
blr = BLR(sys, theta_mean, Sigma_theta, data, features_blr);

% allocate state and input trajectories
x = zeros(nrSteps+1,size(x_0,1),nrTraj); 
u = zeros(nrSteps,nrTraj);
x(1,:,:) = repmat(x_0,[1,1,nrTraj]);

% allocate theta trajectories
theta_kf = zeros(nrSteps,size(theta_mean,1),nrTraj);
theta_kf(1,:,:) = repmat(theta_mean,[1,1,nrTraj]);
theta_blr = zeros(nrSteps,size(theta_mean,1),nrTraj);
theta_blr(1,:,:) = repmat(theta_mean,[1,1,nrTraj]);

% allocate sigma trajectories
Sigma_kf = zeros(nrSteps,size(Sigma_theta,1),size(Sigma_theta,2),nrTraj);
Sigma_kf(1,:,:,:) = repmat(Sigma_theta,[1,1,1,nrTraj]);
Sigma_blr = zeros(nrSteps,size(Sigma_theta,1),size(Sigma_theta,2),nrTraj);
Sigma_blr(1,:,:,:) = repmat(Sigma_theta,[1,1,1,nrTraj]);

% control-loop
for i=1:nrTraj
    for j=1:nrSteps
        [sol, ~, errmsg] = ctrl.solve(x(j,:,i)', {}, 0);
        if errmsg
            error(errmsg);
        end
        u(j,i) = sol(1);
        x(j+1,:,i) = sys_true.step(x(j,:,i)', u(j,i));
        
        % parameter estimation
        data.x = x(j,:,i)';
        y = x(j+1,:,i)' - A_0*x(j,:,i)' - sys_true.B*u(j,i);
        data.y = y(2);
        
        kf.update_estimate(data);
        theta_kf(j+1,:,i) = kf.theta';
        Sigma_kf(j+1,:,:,i) = kf.Sigma;
        
        blr.update_estimate(data);
        theta_blr(j+1,:,i) = blr.theta';
        Sigma_blr(j+1,:,:,i) = blr.Sigma;
    end
end

% plot results
params.plot.color = [0.7216, 0.1490, 0.0039];
params.plot.alpha = 1; params.plot.lw = 1;
plot_x('state-time', figIdx(8), x, sys_true.X, params.plot);
plot_x('state-state', figIdx(9), x, sys_true.X, params.plot);
plot_u(figIdx(10), u, sys_true.U, params.plot);

%% plot parameter estimation evolution
Sigma_est = squeeze(Sigma_blr(30,:,:,:));
theta_est = squeeze(theta_blr(30,:,:))';

params.plot.title='Confidence regions using BLR';
params.plot.posterior = true;
plot_PE6(figIdx(11), Sigma_est, theta_est, [sys_true.params.d; sys_true.params.c], p_tilde, Omega+theta_mean, params.plot)
plot_PE6(figIdx(11), Sigma_theta, theta_mean, [sys_true.params.d; sys_true.params.c], p_tilde, Omega, params.plot, false)

Sigma_est = squeeze(Sigma_kf(30,:,:,:));
theta_est = squeeze(theta_kf(30,:,:))';

params.plot.title='Confidence regions using KF';
plot_PE6(figIdx(12), Sigma_est, theta_est, [sys_true.params.d; sys_true.params.c], p_tilde, Omega+theta_mean, params.plot)
plot_PE6(figIdx(12), Sigma_theta, theta_mean, [sys_true.params.d; sys_true.params.c], p_tilde, Omega, params.plot, false)
