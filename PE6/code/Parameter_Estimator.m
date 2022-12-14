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

classdef Parameter_Estimator  < handle
    %PARAMETER_ESTIMATOR Class for parameter estimation
    %   Class to determine parameters given inputs
    
    properties
        params % struct of parameters
    end
    
    methods
        function obj = Parameter_Estimator(sys)
            %PARAMETER_ESTIMATOR Construct an instance of this class
            %   Construct parameter estimator
            
            %%% Parse inputs %%%
            switch nargin         
                case 1
                    if ~isa(sys, 'System')
                        sys.params = sys;
                    end
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            % initialize parameters
            obj.params = sys.params;
        end       
    end
end

