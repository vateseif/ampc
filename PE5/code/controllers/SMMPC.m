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

classdef SMMPC < Controller
    %SMMPC SM-MPC Controller Class
    %   Construct and solve RAMPC Problem
    
    properties
        sys
        K
    end
    
    methods
        function obj = SMMPC(sys, params)
            %SMMPC Construct an instance of this class
            %   Construct SM-MPC Class and initialize solver
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            
            %%%
            
            obj@Controller(sys, params);
            
            obj.sys = sys;
            
            % look up system dimensions
            n = obj.sys.n;
            m = obj.sys.m;
            N = obj.params.N;
            
            % look up costs
            Q = obj.params.Q;
            R = obj.params.R;
            
            % look up dynamic matrices
            % the LinearAffineSystem class already provides A & B computed with the initial parameter estimate
            A = obj.sys.A; % A_\bar{\theta}
            B = obj.sys.B;
            
            % compute tube & terminal controller K plus terminal cost P
            [K, P] = obj.compute_controller(sys, 0.95, 0.95); % please don't change this
            obj.K = K;
            
            %%% TODO:
            % Implement the SMMPC method as shown in the exercise sheet.
            % For the terminal constraint, please see comment below.
            
            % --------- Start Modifying Code Here -----------
            % use the variables defined below
            z = sdpvar(n, N+1); %nominal state
            v = sdpvar(m, N); %nominal input
            x_hat = sdpvar(n, N+1); %performance state
            u_hat = sdpvar(m, N); %performance input
            x_0 = sdpvar(n, 1); %initial state
            hx_tight = sdpvar(size(obj.sys.X.A,1), N);  %b vectors of state constraints
            hu_tight = sdpvar(size(obj.sys.U.A,1), N);  %b vectors of input constraints
            A_theta = sdpvar(n, n); %A_hat{\theta}_k Matrix
            
            % dynamics
            constraints = [x_hat(:,1)==x_0, z(:,1)==x_0];
            for i=1:N
                % nominal dynamics
                constraints = [constraints z(:,i+1)==A*z(:,i)+B*v(:,i)];
                % control input
                constraints = [constraints u_hat(:,i)==K*(x_hat(:,i) - z(:,i)) + v(:,i)];
                % dynamics
                constraints = [constraints x_hat(:,i+1)==A_theta*x_hat(:,i)+B*u_hat(:,i)];
            end
            % tightening constraints
            for i=1:N
                constraints = [constraints obj.sys.X.A*z(:,i)<=hx_tight(:,i)];
                constraints = [constraints obj.sys.U.A*v(:,i)<=hu_tight(:,i)];
            end
            % terminal constraint (schur complement)
            constraints = [constraints [1, z(:,N+1)'*P; P*z(:,N+1), P]>=zeros(3)];

            objective = 0;
            for i=1:N
                objective = objective + x_hat(:,i)'*Q*x_hat(:,i) + u_hat(:,i)'*R*u_hat(:,i);
            end
            objective = objective + x_hat(:,N+1)'*P*x_hat(:,N+1);
            
            %%% Terminal constraint
            % the tightened terminal constraint can be directly implemented
            % using:
            % z_N \in X_f - F_N^k = { z | z' P z <= 1}
            % where we provide P.
            % If you implement the constraint directly as shown above you
            % HAVE to use the solver 'mosek-socp'. If you want to use
            % 'sedumi' or standard 'mosek' you need to apply the Schur
            % complement to the constraint.
         
            % Optimizer allows to solve optimisation problem repeatedly in a fast
            % manner. Inputs are x_0, A_theta, hx_tight, hu_tight
            % To speed up the solve time, MOSEK can be used with an academic license.
            % Only uncomment the mosek line if you have mosek installed!
            settings = sdpsettings('verbose',1,'solver','sedumi');
            %settings = sdpsettings('verbose',1,'solver','mosek');
            
            % --------- Stop Modifying Code Here -----------
            
            obj.prob=optimizer(constraints, objective, settings, {x_0, A_theta, hx_tight, hu_tight}, {v(:,1)});
        end
        
        function [F, hx_tight, hu_tight] = compute_robust_tightening(obj, K, W)
            %COMPUTE TIGHTENING Computes the disturbance reachable sets (DRS)
            %   and the corresponding tightenings.
            
            %%% Parse inputs %%%
            switch nargin                 
                case 3
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%
            
            % look up horizon length
            N = obj.params.N;
            
            % look up system matrices
            % the LinearAffineSystem class already provides A & B computed with the initial parameter estimate
            A = obj.sys.A;
            B = obj.sys.B;
            
            % look up constraints
            X = obj.sys.X;
            U = obj.sys.U;
            
            %%% TODO:
            % Implement the computation of the DRS and the tightenings
            % below. The matrix A defined above is already A_\bar{\theta}
            % Please make sure you output hx_tight and hu_tight such that
            % their dimensions agree with the dimensions you defined in the
            % optimization problem!
            
            % --------- Start Modifying Code Here -----------
            % constraint tightening
            F = {(A+B*K)^0 * W};
            PX = X-F{1}; PX=PX.minHRep();
            PU = U-K*F{1}; PU=PU.minHRep();
            hx_tight = zeros(4, N); hx_tight(:,1)=PX.b;
            hu_tight = zeros(2, N); hu_tight(:,1)=PU.b;
            for i=2:N
                F{i} = F{i-1} + (A+B*K)^i * W;
                % NOTE: matlab is SHIT and size of PX randomly changes
                % using minHRep seems to fix the issue
                PX = X-F{i}; PX=PX.minHRep();
                PU = U-K*F{i}; PU=PU.minHRep();
                hx_tight(:,i) = PX.b;
                hu_tight(:,i) = PU.b;
            end

            
            % --------- Stop Modifying Code Here -----------
        end
        
        function [K, P] = compute_controller(obj, sys, rho, lambda)
            %COMPUTE_TUBE Computes tube & terminal controller and terminal
            %cost. The method is from Appendix A in Köhler et al. (2019),
            %"Linear robust adaptive model predictive control:
            %Computational complexity and conservatism"
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    rho = 0.9;
                    lambda = 0.9;
                    
                case 3
                    lambda = 0.9;
                    
                case 4
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            
            %%%
            
            % define verteces of Omega
            vertices_theta = sys.Omega.V';
            
            % look up necessary values
            n = size(sys.A,2);
            m = size(sys.B,2);
            B = sys.B;
            Q = obj.params.Q;
            R = obj.params.R;
            W = sys.W;
            
            % Computation of a rho-contractive Polytope
            % Find feedback K and terminal cost P
            E = sdpvar(n,n);
            Y = sdpvar(m,n);
            
            % objective
            objective = -logdet(E);
            
            % constraints
            constraints=[];
            for i=1:size(vertices_theta,1)
                A = sys.A_delta{1};
                for j=2:length(sys.A_delta)
                    A = A + sys.A_delta{j}*vertices_theta(i,j-1);
                end

                % Lyapunov equation
                constraints=[constraints,...
                    [E, (A*E+B*Y)', Q^0.5*E, Y'*R^0.5;
                    (A*E+B*Y), E, zeros(n), zeros(n,m);
                    (Q^0.5*E)', zeros(n), eye(n), zeros(n,m);
                    (Y'*R^0.5)', zeros(m,n), zeros(m,n), eye(m)]>=0];
                % rho-contractivity
                constraints=[constraints,...
                    [rho*E,(A*E+B*Y)';
                    (A*E+B*Y),rho*E]>=0];
                % RPI condition
                for j=1:size(W.V,1)
                    constraints=[constraints,...
                        [lambda*E, zeros(n,1), (A*E+B*Y)';
                        zeros(1,n), 1-lambda, W.V(j,:);
                        (A*E+B*Y), W.V(j,:)', E]>=0];
                end
            end
            
            % Constraint satisfaction
            F=[sys.X.A./sys.X.b;zeros(size(sys.U.A,1),n)];
            G=[zeros(size(sys.X.A,1),m);sys.U.A./sys.U.b];
            for i=1:size(sys.X.A,1)+size(sys.U.A,1)
                constraints=[constraints,...
                    [1,F(i,:)*E+G(i,:)*Y;
                    (F(i,:)*E+G(i,:)*Y)',E]>=0];
            end
            
            % solve
            optimize(constraints, objective, sdpsettings('solver', '', 'verbose', 0));
            P = inv(value(E));
            K = value(Y)*P;
        end
    end
end

