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
classdef Nonlinear_RMPC < Controller
    %NLMPC Nonlinear MPC Controller Class
    %   Construct and solve nominal nonlinear MPC Problem
    
    properties
        % we need to define the following properties, for Ipopt to work
        % properly as a class object.
        x %state (optimization variable)
        x_0 %initial state (parameter)
        u %input (optimization variable)
        solver %NLP solver name
        
        % since this controller uses feedback, we also store K
        K % feedback gain
        
        % since we need to compute the RPI set, we additionally store the
        % system information
        sys % System object
        
        diff_sys % differential dynamics
    end
    
    methods
        function obj = Nonlinear_RMPC(sys, params, diff_sys, rho, w_bar, solver)
            %NRMPC Construct an instance of this class
            %   Construct non-linear robust MPC Class and initialize solver
            
            %%% Parse inputs %%%
            switch nargin
                case 3
                    rho = 0.8;
                    w_bar = [];
                    solver = 'ipopt';
                    
                case 4
                    w_bar = [];
                    solver = 'ipopt';
                    
                case 5
                    solver = 'ipopt';
                    
                case 6

                otherwise 
                    error('Wrong number of inputs!')
            end
            
            %%%
            
            % call super class constructor
            obj@Controller(sys, params);
            
            % store system information
            obj.sys = sys;
            
            % store differential system information
            obj.diff_sys = diff_sys;
            
            % initialize NLP solver name
            obj.solver = solver;
            
            %%% Initialize the non-linear MPC Controller
            % --- compute tightening ---
            [c_x, c_u, P, K, delta] = obj.compute_tightening(rho);
            % store K
            obj.K = K;
            
            % --- compute L_w & reset delta ---
            if ~isempty(w_bar)
                %%% TODO %%%
                % compute L_w and recompute delta according to manually set
                % w_bar in this if clause.
            end
                        
            % --- initialize Opti stack ---
            obj.prob = casadi.Opti();
            
            % --- define optimization variables ---
            obj.x = obj.prob.variable(sys.n,obj.params.N+1);
            obj.x_0 = obj.prob.parameter(sys.n,1);
            obj.u = obj.prob.variable(sys.m,obj.params.N);
            
            %%% TODO %%%
            % Implement the nonlinear robust MPC problem.
            % Make sure you define obj.K!
            % Implement the state dependent avoidance constraint using the
            % same if else construction as above, i.e.,
            % if ~isempty(w_bar)
            %      constraint (you may have to rewrite the constraint to
            %                  avoid nummerical issues with ipopt)
            % end
            
            % --- start inserting here --- 
            
            
            % --- stop inserting here ---
            %%%
            
            % --- setup NLP ---
            % initialize non-verbose
            obj.prob.solver(solver, struct('print_time', 0), struct('print_level', 0));          
        end
        
        function [c_x, c_u, P, K, delta, w_bar] = compute_tightening(obj, rho)
            %COMPUTE TIGHTENING Computes an RPI set and the corresponding
            %   tightening, which minimizes the constraint tightening.
            
            %%% Parse inputs %%%
            switch nargin
                case 1
                    rho = 0.8;
                    
                case 2
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%
            
            % define differential system matrices
            B = obj.diff_sys.B{1};
            A_1 = obj.diff_sys.A{1};
            A_2 = obj.diff_sys.A{2};
            
            %%% TODO %%%
            % Define the SDP for computing a contractive ellipsoid
            % Compute w_bar and the sublevel set delta, which renders the
            % ellipsoid RPI for w_bar as well as the corresponding state
            % and input constraint tightenings.
            %
            % We recommend weighing the state tightenings in the objective
            % function by a factor of 50, i.e., 50*sum(gamma_x) in order
            % to obtain good results. However, any RPI set which you
            % compute with the provided constraints in 1a) and which 
            % results in a recursively feasible RMPC scheme, as well as 
            % feasible for the provided initial condition will be accepted.
            % --- start inserting here ---     

            % use the following definition as the vertices of the
            % disturbance set.
            W_V = obj.params.G*obj.sys.X.V';
            
            constraints = ;
            objective = ;
            
            % solve problem
            options = sdpsettings('solver','sedumi', 'verbose', 0);
            % only uncomment if you have Mosek installed
            %options = sdpsettings('solver','mosek', 'verbose', 0);
            
            % --- define optimizer object ---
            optimize(constraints, objective, options);
            
            
            % --- define function outputs ---
            % recover Lyapunov function & controller
            P = ;
            K = ;
            
            %--- compute tightening ---
            % compute w_bar & delta
            w_bar = ;
            delta = ;
            
            % compute tightening of state constraints
            c_x = ;
            
            % compute tightening of input constraints
            c_u = ;
            
            % --- stop inserting here --- 
        end
    end
end
