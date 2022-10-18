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

classdef RMPC < Controller
    %RMPC Robust MPC Controller Class
    %   Construct and solve robust linear MPC Problem
    
    properties
        solver % solver name
        
        % since this controller uses feedback, we also store K
        K % feedback gain
        
        sys %System description
    end
    
    methods
        function obj = RMPC(sys, params, rho, solver)
            %   RMPC Construct an instance of this class
            %   Construct the robust MPC Class and
            %   initialize solver
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    rho = 0.9;
                    solver = 'sedumi';
                    
                case 3
                    solver = 'sedumi';
                    
                case 4
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            
            %%%
            
            % call super class constructor
            obj@Controller(sys, params);
            
            % Internal system description
            obj.sys = sys;
            
            % initialize solver name
            obj.solver = solver;
            
            %%% Initialize the RMPC Controller
            
            % --- define optimization variables ---
            z = sdpvar(obj.sys.n, obj.params.N+1); % Nominal prediction states
            x_k = sdpvar(obj.sys.n, 1); % Current true state, which is the input to the obj.prob optimizer object
            v = sdpvar(obj.sys.m, obj.params.N);   % Nominal prediction inputs
            
            %%% TODO %%%
            % Compute the required tightenings given rho
            % You have to define obj.K!
            % Define the robust MPC problem constraints and objective.
            
            % --- start inserting here --- 
            
            % compute tightenings
            [x_tight, u_tight, P, K, delta] = obj.compute_tightening(rho);
            obj.K = K;
            X_tight = Polyhedron(sys.X.A, sys.X.b - x_tight);
            U_tight = Polyhedron(sys.U.A, sys.U.b - u_tight);

            % objective
            objective = 0;
            for i=1:obj.params.N
                objective = objective + z(:,i)'*obj.params.Q*z(:,i) + v(:,i)'*obj.params.R*v(:,i);
            end
            % dynamics
            constraints = [];
            for i = 1:obj.params.N
                constraints = [constraints, z(:,i+1)==obj.sys.A*z(:,i)+obj.sys.B*v(:,i)];
            end
            % state tightening constraints
            for i=1:obj.params.N+1
                constraints = [constraints, ismember(z(:,i), X_tight)];
                %constraints = [constraints, sys.X.A*z(:,i)<=sys.X.b - x_tight];
            end
            constraints = [constraints, z(:,end) == [0; 0]];
            % input tightening constraints
            for i = 1:obj.params.N
                constraints = [constraints, ismember(v(:,i), U_tight)];
                %constraints = [constraints, sys.U.A*v(:,i)<=sys.U.b - u_tight];
            end
            % initial state constraint
            constraints = [constraints, (x_k-z(:,1))'*P*(x_k-z(:,1)) <= delta^2];

            % --- stop inserting here ---
            %%%
            
            % --- setup Yalmip solver object ---
            obj.prob=optimizer(constraints, objective, sdpsettings('solver',solver), {x_k}, {v(:,1),z(:,1)});    
        end
        
        function [x_tight, u_tight, P, K, delta] = compute_tightening(obj, rho)
            %COMPUTE TIGHTENING Computes an RPI set and the corresponding
            %   tightening, which minimizes the constraint tightening.
            
            %%% Parse inputs %%%
            switch nargin
                case 1
                    rho = 0.9;
                    
                case 2
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%
                       
             
            %--- setup & solve offline optimization problem ---
            
            %%% TODO %%%
            % Define the SDP for computing a contractive ellipsoid
            % Compute the sublevel set delta which renders the ellipsoid
            % RPI as well as the corresponding state and input constraint
            % tightenings. 
            %
            % Make sure to define every output of the function!
            %
            % We recommend weighing the state tightenings in the objective
            % function by a factor of 50, i.e., 50*sum(c_{x,j}^2) in order
            % to obtain good results. However, any RPI set which you
            % compute with the provided constraints in 1a) and which 
            % results in a recursively feasible RMPC scheme, as well as 
            % feasible for the provided initial condition will be accepted.
            % --- start inserting here ---
            
            % params
            n = obj.sys.n;
            m = obj.sys.m;
            nx = length(obj.sys.params.b_x);
            nu = length(obj.sys.params.b_u);
            A = obj.sys.params.A;
            B = obj.sys.params.B;
            Ax = obj.sys.params.A_x;
            Au = obj.sys.params.A_u;
            v_w = obj.sys.params.A_w \ obj.sys.params.b_w; %% TODO: is this right?
            % init decision variables 
            Y = sdpvar(m, n, 'full');
            E = sdpvar(n, n);
            c_x_2 = sdpvar(nx,1); % vector of values c_{x,j}^2
            c_u_2 = sdpvar(nu,1); % vector of values c_{u,j}^2
            w_b_2 = sdpvar(1);  % w_bar^2
            % objective 
            cost = 0.5/(1-rho) * ((nx + nu) * w_b_2 + 50 * sum(c_x_2) + sum(c_u_2));
            % constraints
            epsilon = 0; % tolerance for converting > to >=
            con = E >= eye(n);
            con = [con, [rho^2*E, (A*E+B*Y)'; (A*E+B*Y), E] >= eye(2*n) * epsilon];
            for j=1:nx
                con = [con, [c_x_2(j), Ax(j,:)*E; E'*Ax(j,:)', E] >= eye(n+1) * epsilon];
            end
            for j=1:nu
                con = [con, [c_u_2(j), Au(j,:)*Y; Y'*Au(j,:)', E] >= eye(n+1) * epsilon];
            end
            con = [con, [w_b_2, v_w'; v_w, E] >= eye(n+1)*epsilon];

            
            options = sdpsettings('solver','sedumi', 'verbose', 0);
            % Only uncomment if you have Mosek installed, it will result in
            % a nice speedup
            %options = sdpsettings('solver','mosek', 'verbose', 0);
            
            % run optimizer
            optimize(con, cost, options);
            % retrieve solutions
            P = inv(value(E));
            K = value(Y) / value(E);
            c_x = sqrt(value(c_x_2));
            c_u = sqrt(value(c_u_2));
            w_b = sqrt(value(w_b_2));
            x_tight = c_x * w_b / (1 - rho);
            u_tight = c_u * w_b / (1 - rho);
            delta = w_b / (1-rho);


            % --- stop inserting here ---
            %%%
            
        end
    end
end

