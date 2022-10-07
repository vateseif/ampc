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

classdef NonlinearSystem < System
    %SYSTEM System class implementing a segway system
    %   The System class defines the interfaces, which will be used by the
    %   controller and parameter classes implemented in the recitations.
    properties
        f %system dynamics
        h %output dynamics
    end
    methods
        function obj = NonlinearSystem(params)
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
            
            % call parent class constructor
            obj@System(params);

            % define update law
            obj.f = @(x,u) obj.params.f(x,u) + obj.generateNoise(obj.noiseArgs{:});
            obj.h = @(x,u) obj.params.h(x,u);
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
           
            % call parent class constructor
            update_params@System(params);

            % define update law
            obj.f = @(x,u) obj.params.f(x,u) + obj.generateNoise(obj.noiseArgs{:});
            obj.h = @(x,u) obj.params.h(x,u);
        end
    end
end