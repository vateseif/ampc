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

classdef Nonlinear_MPC < Controller
    %NLMPC Nonlinear MPC Controller Class
    %   Construct and solve nominal nonlinear MPC Problem
    
    properties
        % we need to define the following properties, for Ipopt to work
        % properly as a class object.
        x %state (optimization variable)
        x_0 %initial state (parameter)
        u %input (optimization variable)
        solver %NLP solver name
        objective %nominal nonlinear MPC objective function
    end
    
    methods
        function obj = Nonlinear_MPC(sys, params, solver)
            %NLMPC Construct an instance of this class
            %   Construct non-linear MPC Class and initialize solver
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    solver = 'ipopt';
                    
                case 3
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            
            %%%
            
            % call super class constructor
            obj@Controller(sys, params);
            
            % initialize NLP solver name
            obj.solver = solver;
            
            %%% TODO %%%
            % Define the nonlinear nominal MPC problem below.
            % Use the Opti stack from CasADi to formulate the optimization
            % problem.
            % You have to define obj.prob!
            
            % --- start inserting here ---
            
            % init
            prob = casadi.Opti();
            N = obj.params.N;
            % variables
            obj.x = prob.variable(2, N+1);
            obj.u = prob.variable(1, N+1);
            % params
            obj.x_0 = prob.parameter(2,1);
            % cost
            cost = 0;
            for i = 1:N
                cost = cost + obj.x(:, i)' * obj.params.Q * obj.x(:, i) + obj.u(i)^2 * obj.params.R;
            end
            obj.objective = cost;
            % initial condition
            prob.subject_to(obj.x(:,1) == obj.x_0);
            % terminal condition
            prob.subject_to(obj.x(:,end) == [0; 0]);
            % system dynamics
            for i = 1:N
                prob.subject_to(obj.x(:,i+1) == sys.f(obj.x(:,i), obj.u(i)));
            end
            % state & input constraints
            for i = 1:N
                prob.subject_to(sys.params.A_x * obj.x(:,i) <= sys.params.b_x);
                prob.subject_to(sys.params.A_u * obj.u(:,i) <= sys.params.b_u);
            end
            % min cost
            prob.minimize(cost);
            obj.prob = prob;
            
            % --- stop inserting here ---
            %%%
            
            % --- setup NLP ---
            % initialize non-verbose
            obj.prob.solver(solver, struct('print_time', 0), struct('print_level', 0));          
        end
        
    end
end


