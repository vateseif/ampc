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

classdef Indirect_Feedback_SMPC < Controller
    %Indirect_Feedback_SMPC Controller Class
    %   Construct and solve indirect feedback SMPC problem
    
    properties
        sys % system information
    end
    
    methods
        function obj = Indirect_Feedback_SMPC(sys, params, K, P, solver)
            %Indirect_Feedback_SMPC Construct an instance of the Indirect_Feedback_SMPC class
            %   initialize SMPC optimization problem and define p
            %   property
            
            %%% Parse inputs %%%
            switch nargin  
                case 2
                    [K,P] = dlqr(sys.A,sys.B,params.Q,params.R);
                    K=-K;
                    solver = '';
                    
                case 3
                    [~,P] = dlqr(sys.A,sys.B,params.Q,params.R);
                    solver = '';
                    
                case 4
                    solver = '';
                    
                case 5
                    
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%
            
            % call parent class constructor
            obj@Controller(sys, params);
            
            obj.sys = sys;
            
            %%% TODO: Implement the indirect feedback SMPC%%%
            %%% Start inserting code here %%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% --- Initialize MPC Controller ---
            % define optimization variables
            x_bar = [];
            z = [];
            e_bar = [];
            x_k = []; % true state x(k); input to the optimizer
            z_1 = []; % last predicted nominal state z_1|k-1; input to the optimizer
            v = [];
            u_bar = [];
            xtight = sdpvar(size(X.A,1), obj.params.N); % These are the state tightenings which will be provided as an input to the optimizer, i.e. in the halfspace formulation: A_x*x <= b_x - xtight
            utight = sdpvar(size(U.A,1), obj.params.N); % These are the input tightenings which will be provided as an input to the optimizer, i.e. in the halfspace formulation: A_u*u <= b_u - utight
            
            objective = 0;
            constraints = [];
            
            %%% Stop inserting code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % setup Yalmip solver object
            % Inputs: current state x(k) in x_bar_0, last predicted state 
            % z_1|k-1 in z_0 and state and input tightenings
            % Outputs: the expected input u_bar_0 and the expected state at
            % the first predicted time step x_bar_1
            obj.prob=optimizer(constraints, objective, sdpsettings('solver',solver), {x_k, z_1, xtight, utight}, {u_bar(:,1), z(:,2), objective});
        end
        
       
        function [xtight, utight, F, p_tilde, K] = compute_PRS_min(obj, p, N, solver)
            %COMPUTE PRS MIN Computes PRS sets and the corresponding
            %   tightening using an optimisation problem.
            % Returns cells xtight and utight with the constraint tightenings
            % for time steps 1 to N and terminal tightening at index N+1 and
            % a cell P where the PRS Sets for time steps 1 to N are given
            % as well as the Chebyshev Reachable Set for all time steps
            % at index N+1
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    N = 0;
                    solver = '';
                    
                case 3
                    solver = '';
                    
                case 4
                    
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%
            
            % look up system parameters
            dt = obj.sys.params.dt; %sampling time
            k = obj.sys.params.k; %spring constant
            c = obj.sys.params.c; %damping constant
            l = obj.sys.params.l; %length
            g = obj.sys.params.g; %gravitational constant
            n = obj.sys.n; %state dimension
            m = obj.sys.m; %input dimension
            noiseCovariance = obj.sys.params.noiseCovariance; %noise covariance
            
            % define system matrices
            A = obj.sys.A;
            B = obj.sys.B;
            
            % Computation of p_tilde
            p_tilde = chi2inv(p, n);
            
            % Compute tightening according to SDP
            E = sdpvar(n,n);
            Y = sdpvar(m,n);
            
            %%% TODO: Implement SDP for infinite step PRS Computation %%%
            %Hint: Use the Lyapunov type inequality seen in the Recitation.
            %You can use trace(.) as a volume upper bound.
            
            %%% Start inserting code here %%%%%%%%%%%%%%%%%%%%%%%%%%
        
            objective = 0;
            constraints = [];
            
            %%% Stop inserting code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            optimize(constraints, objective, sdpsettings('verbose',0,'solver',solver));
            
            % The infinite step PRS is stored in F{N+1}. We also store
            % the corresponding controller.
            F{N+1}=inv(value(E));
            K = value(Y)*F{N+1};
            
            % Computation of p_tilde
            p_tilde = chi2inv(p, n);

            % closed loop
            A_K = A+B*K;
            
            %%% TODO: Compute the error variance  %%%
            %%% Start inserting code here %%%%%%%%%%%%%%%%%%%%%%%%%%
           
            var_e{1} = zeros(n);
            for i = 1:N
               % Use the closed-loop dynamics to propagate the error
               % variance as seen in the recitation.
               var_e{i+1} = []; 
            end
            
            %%% Stop inserting code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Store the PRS shapes in F
            for i = 1:N
                F{i} = inv(var_e{i+1});
            end
            
            %--- compute tightening ---
            % look up state, input, and disturbance sets
            X = obj.sys.X;
            U = obj.sys.U;

            %e = sdpvar(n,1);
            
            xtight(:,1)=zeros(size(X.b));
            utight(:,1)=zeros(size(U.b));     
            % for every time step
            for i=1:N+1
                % for every state constraint
                for j=1:length(X.b)
                    xtight(j,i+1)=norm(F{i}^(-0.5)*X.A(j,:)',2)*sqrt(p_tilde);                
                end
                for j=1:length(U.b)
                    utight(j,i+1)=norm(F{i}^(-0.5)*K'*U.A(j,:)',2)*sqrt(p_tilde);
                end
            end
            
            for i=1:N+1
                if ~all(X.b-xtight(:,i)>=0) && ~all(U.b-utight(:,i)>=0)
                    error('Infinite Step PRS Set is bigger than the state constraints')
                end
            end
        end
    end
end

