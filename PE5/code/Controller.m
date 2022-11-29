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

classdef Controller < handle
    %CONTROLLER Abstract parent class for controller implementation
    %   Class to determine the input of a system given its current state
    
    properties
        params  %struct of parameters
        prob    %Yalmip optimizer or CasADi solver object
    end
    
    methods
        function obj = Controller(sys, params)
            %CONTROLLER Construct an instance of this class
            %   Constructs a controller for a given system
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            % Initialize properties
            obj.params = params;
            obj.prob = [];
        end
        
        
        function [u, out, info] = solve(obj, x, vars, verbose)
            %SOLVE Solve the optimisation problem
            %   Solves the optimization problem and returns the input to be
            %   applied and additional outputs
            
            %%% Parse input arguments %%%
            switch nargin
                case 2
                    vars = {};
                    verbose = 0;
                    
                case 3
                    verbose = 0;
                    
                case 4
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            if ~isempty(obj.prob)
                if isa(obj.prob, 'optimizer')
                    % YALMIP optimizer
                    
                    % change the verbosity of the problem
                    obj.prob.options.verbose = verbose;
                    % construct input arguments for the optimizer object
                    if ~isempty(vars)
                        inputs={x,vars{:}};
                    else
                        inputs=x;
                    end
                    % solve the optimizer
                    [sol, errcode] = obj.prob(inputs);
                    % check for errors
                    if errcode
                        info = yalmiperror(errcode);
                    else
                        info = false;
                    end
                    % parse optimizer outputs
                    if iscell(sol)
                        u = sol{1};
                        out = {sol{2:end}};
                    else
                        u = sol;
                        out = [];
                    end
                    
                else
                    if isa(obj.prob, 'casadi.Opti')
                        % CasADi Opti Stack
                        
                        % set verbosity level
                        persistent verbosity
                        if isempty(verbosity)
                            verbosity = 0;
                        end
                        if xor(verbosity, verbose)
                            verbosity = verbose;
                            obj.prob.solver(obj.solver, struct('print_time', verbose), struct('print_level', verbose*5));
                        end
                        
                        % set initial state
                        obj.prob.set_value(obj.x_0, x);
                        
                        % solve the optimization problem
                        sol = obj.prob.solve();
                        
                        % parse optimizer outputs and errors
                        if sol.stats.success
                            info = false;
                            u = sol.value(obj.u);
                            out = sol.value(obj.x);
                        else
                            info = sol.stats.return_status;
                            out = [];
                            u = [];
                        end
                        
                    else
                        error('Optimization problem type not supported!')
                    end
                end
            else
                error('Solver not initialised!')
            end
        end
    end
end