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

classdef System < handle
    %SYSTEM System class implementing a segway system
    %   The System class defines the interfaces, which will be used by the
    %   controller and parameter classes implemented in the recitations.
    properties
        params %parameter struct
        X %MPT Polytope object describing the state constraints
        U %MPT Polytope object describing the input constraints
        W %MPT Polytope object describing the disturbance set
        generateNoise %stochastic process according which disturbances are drawn
        noiseArgs %arguments necessary for stochastic disturbance process
        n %state dimension
        m %input dimension
        dt %discretization step
    end
    methods
        function obj = System(params)
            %SYSTEM System Class Constructor
            %   constructs system object according to the provided
            %   parameters
            
            %%% Parse inputs %%%
            switch nargin
                case 1
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            obj.params = params;
            
            % define system constraints as Polytopes
            obj.X = Polyhedron(obj.params.A_x, obj.params.b_x);
            obj.U = Polyhedron(obj.params.A_u, obj.params.b_u);
            if ~isempty(obj.params.A_w) &&  ~isempty(obj.params.b_w)
                obj.W = Polyhedron(obj.params.A_w, obj.params.b_w);
            end

            % get noise distribution & linearity of the system
            obj.generateNoise = obj.params.generateNoise;
            obj.noiseArgs = obj.params.noiseArgs;
            
            % get system dimensions
            obj.n = obj.params.n;
            obj.m = obj.params.m;
        end
        
        function x1 = step(obj,x,u)
            %STEP Advance system from state x with input u
            
            %%% Parse inputs %%%
            switch nargin
                case 3
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            x1 = obj.f(x,u);
        end
        
        function y1 = get_output(obj,x,u)
            %GET_OUTPUT Evaluate output function for state x and input u
            
            %%% Parse inputs %%%
            switch nargin
                case 3
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            y1 = obj.h(x,u);
        end
        
        function update_params(obj, params)
            %UPDATE_PARAMS
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
           
            obj.params = params;
            
            % define system constraints as Polytopes
            obj.X = Polyhedron(obj.params.A_x, obj.params.b_x);
            obj.U = Polyhedron(obj.params.A_u, obj.params.b_u);
            if ~isempty(obj.params.A_w) &&  ~isempty(obj.params.b_w)
                obj.W = Polyhedron(obj.params.A_w, obj.params.b_w);
            end

            % get noise distribution & linearity of the system
            obj.generateNoise = obj.params.generateNoise;
            obj.noiseArgs = obj.params.noiseArgs;

            
            % get system dimensions
            obj.n = obj.params.n;
            obj.m = obj.params.m;
            
            % get sampling time
            obj.dt = obj.params.dt;
        end
    end
end