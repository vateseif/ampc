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

classdef GP < Parameter_Estimator
    %GP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        data
        kernel
    end
    
    methods
        function obj = GP(sys, data, kernel)
            %GP Construct an instance of this class
            %   Construct GP Class. The second
            %   input is a struct with data.x an nxN input data matrix and
            %   data.y an mxN output data matrix of N data points
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    if size(data.x,2)~=size(data.y,2)
                        error('Wrong data format!')
                    end
                    kernel = @(x,y) x.^2'*y.^2;
                    
                case 3
                    if size(data.x,2)~=size(data.y,2)
                        error('Wrong data format!')
                    end
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            obj@Parameter_Estimator(sys);
            
            obj.kernel = kernel;
            
            obj.data = data;
            
        end
                
        function [y, var] = predict(obj, x)
            %PREDICT Predict function value
            %   Predict function value based on input and provided data
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            y = obj.kernel(x, obj.data.x)*inv(obj.kernel(obj.data.x,obj.data.x)+obj.params.noiseCovariance^2*eye(size(obj.data.x,2)))*obj.data.y';
            
            var = obj.kernel(x,x)-obj.kernel(x,obj.data.x)*inv(obj.kernel(obj.data.x, obj.data.x) ...
                  + obj.params.noiseCovariance^2*eye(size(obj.data.x,2)))*obj.kernel(obj.data.x,x);

        end
        
        function add_data(obj, data)
            %ADD_DATA Add data for regression
            %   Adds data to saved data for regression
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    if size(data.x,2)~=size(data.y,2)
                        error('Wrong data format!')
                    end
                    if size(data.x,1) ~= size(obj.data.x,1)
                        error('Wrong input format!')
                    end
                    if size(data.y,1)~= size(obj.data.y,1)
                        error('Wrong output format!')
                    end
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            obj.data.x = [obj.data.x, data.x];
            obj.data.y = [obj.data.y, data.y];
        end
    end
end

