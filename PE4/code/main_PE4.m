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
rng(7);

%% get parameters and define system and controller
params = get_params('params_PE4');
sys = LinearSystem(params.sys);
ctrl_RI = PRS_SMPC(sys, params.ctrl);

%% Exercise 1a)
% Implement compute_PRS_min in the PRS_SMPC class, where we 
% optimise over the shape of the PRS and the tube controller.
p = 0; %TODO: Change p and observe the different tightenings

[xtight, utight, F, p_tilde, K] = ctrl_RI.compute_PRS_min(p, params.sim.nrSteps+ctrl_RI.params.N);

% tightened state constraints with the infinite time step PRS
X_tight = Polyhedron(sys.X.A, sys.X.b - xtight(:, end));

% tightened state constraints with the infinite time step PRS
U_tight = Polyhedron(sys.U.A, sys.U.b - utight(:, end));

figIdx = 1:30;
plot_PE4(figIdx(1), params.sim.nrSteps, X_tight, U_tight, ...
           sys.X, sys.U, F, p_tilde, params.plot)

%% Exercise 1b)
% Implement the PRS tightening SMPC problem in PRS_SMPC.m
% Implement the conditional update rule z_0 = x(k) if feasible, 
% else z_0 = z_1|k-1 
% Then we simulate the closed-loop system for nrSteps time steps and nrTraj
% trajectories starting in x_0.

ctrl_RI = PRS_SMPC(sys, params.ctrl);
nrSteps = params.sim.nrSteps;
nrTraj = params.sim.nrTraj;
x_0 = params.sim.x_0;
z_0 = params.sim.x_0;

% allocate state and input trajectories and cost
x = zeros(nrSteps+1,size(x_0,1),nrTraj); 
x_bar = zeros(nrSteps+1,size(z_0,1),nrTraj); 
u = zeros(nrSteps,nrTraj);
cost = zeros(nrSteps, nrTraj);
x(1,:,:) = repmat(x_0,[1,1,nrTraj]);
x_bar(1,:,:) = repmat(z_0,[1,1,nrTraj]);
rng(7);

% control-loop
for j=1:nrTraj
    for k=1:nrSteps
        
        %%% TODO %%%
   
        % --- start inserting here ---            
            % Solve  the optimization problem with the true state
            % x_bar_0=x(k).
            %
            % The first input of ctrl_RI.solve is the initial state x_bar_0
            % of the optimization problem, which you need to define.
            % Hint: You can check the state update in this control loop to
            % see how to access the current state of the current
            % trajectory.
            %
            % The second input of ctrl_RI.solve is a cell containing all
            % additional inputs to the optimizer object, i.e. the
            % tightening of the state and input constraints with respect to
            % the computed PRS Sets F^p_{k}. Note that at every time step
            % k, we need the tightenings for F^p_{k+i} for i=0,...,N-1 
            % in the optimization, which corresponds to
            % xtight(:,k:k+ctrl_RI.params.N-1). You do not need to change
            % these.
            [u_bar, x_bar_1, errmsg] = ctrl_RI.solve([], {xtight(:,k:k+ctrl_RI.params.N-1), utight(:,k:k+ctrl_RI.params.N-1)}, 0);
            
            % Check if the optimization problem was infeasible using errmsg.
            % If so, apply the computed input to the system. Otherwise,
            % recompute the optimization problem with x_bar_0=x_bar_1|k-1. 
                 
            if [] % if infeasible or any other issues
                % Similar to before, change the first input to
                % ctrl_RI.solve such that x_bar_0=x_bar_1|k-1.
                % Hint: The previous x_bar_1 are saved in the variable
                % x_bar. Check within this control loop how to correctly
                % access it.
                [u_bar, x_bar_1, errmsg_nom] = ctrl_RI.solve([], {xtight(:,k:k+ctrl_RI.params.N-1), utight(:,k:k+ctrl_RI.params.N-1)}, 0);
 
                % If this is still infeasible, something is wrong.
                if errmsg_nom
                    error(errmsg_nom);
                end
                % Apply the corresponding control law. Make sure you apply
                % the correct feedback!
                u(k,j) = [];
            else
                % If the problem given the true state was feasible, apply 
                % the corresponding input
                u(k,j) = [];
            end
            
        % --- stop inserting here ---
        %%%

        
        x_bar(k+1,:,j) = x_bar_1{1};
        x(k+1,:,j) = sys.step(x(k,:,j)', u(k,j));
        cost(k,j) = x(k,:,j)*ctrl_RI.params.Q*x(k,:,j)'+u(k,j)*ctrl_RI.params.R*u(k,j)';
    end
end
disp('Average incurred cost with recovery initialisation:')
disp(mean(cost(:)))
%% plot results
plot_x('state-time', figIdx(2), x, sys.X, params.plot, '', 'State-time plot');
plot_x('state-state', figIdx(3), x, sys.X, params.plot, '', 'State-state plot');
plot_u(figIdx(4), u, sys.U, params.plot);
plot_cost(figIdx(5), cost, params.plot, '', 'True stage cost');

%% Exercise 1c)
% Implement the indirect feedback SMPC problem in Indirect_Feedback_SMPC.m
% Then we simulate the closed-loop system for nrSteps time steps and nrTraj
% trajectories starting in x_0
% You can copy paste the compute_PRS_min function from the recovery
% initialization into the indirect feedback file.

ctrl_IF = Indirect_Feedback_SMPC(sys,params.ctrl, K);
rng(7);

% control-loop
for j = 1:nrTraj
    for k = 1:nrSteps
        [u_bar, x_bar_1, errmsg] = ctrl_IF.solve(x(k,:,j)',{x_bar(k,:,j)', xtight(:,k:k+ctrl_IF.params.N-1), utight(:,k:k+ctrl_RI.params.N-1)}, 0);
        
        if errmsg
            error(errmsg);
        end
        
        u(k,j) = u_bar;
        x_bar(k+1,:,j) = x_bar_1{1};
        x(k+1,:,j) = sys.step(x(k,:,j)', u(k,j));
        cost(k,j) = x(k,:,j)*ctrl_IF.params.Q*x(k,:,j)'+u(k,j)*ctrl_IF.params.R*u(k,j)';
    end
end
disp('Average incurred cost with indirect feedback:')
disp(mean(cost(:)))

%% plot results
plot_x('state-time', figIdx(6), x, sys.X, params.plot, '', 'State-time plot');
plot_x('state-state', figIdx(7), x, sys.X, params.plot, '', 'State-state plot');
plot_u(figIdx(8), u, sys.U, params.plot);
plot_cost(figIdx(9), cost, params.plot, '', 'True stage cost');