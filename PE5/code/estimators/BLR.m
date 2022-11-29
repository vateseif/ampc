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

classdef BLR < Parameter_Estimator
    %BLR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        theta % current mean parameters estimate
        Sigma % current variance
        features % BLR features
    end
    
    methods
        function obj = BLR(sys, prior_mean, prior_variance, data, features)
            %BLR Construct an instance of this class
            %   Construct Bayesian Linear Regression Class. Data is a struct 
            %   with data.x an nxN input data matrix and data.y an
            %   mxN output data matrix of N data points
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    prior_variance = eye(size(prior_mean));
                    data.x = [];
                    data.y = [];
                    features = @(x) x;
                    
                case 3
                    data.x = [];
                    data.y = [];
                    features = @(x) x;
                    
                case 4
                    if size(data.x,2)~=size(data.y,2)
                        error('Wrong data format!')
                    end
                    features = @(x) x;
                
                case 5 
                    if size(data.x,2)~=size(data.y,2)
                        error('Wrong data format!')
                    end
                    
                    if ~isa(features, 'function_handle')
                        error('Wrong function type for features')
                    end
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            obj@Parameter_Estimator(sys);
            
            obj.features = features;
            
            obj.theta = prior_mean;
            
            obj.Sigma = prior_variance;
            
            if ~isempty(data.x) && ~isempty(data.y)
                Sigma_inv = 1/(obj.params.noiseCovariance)^2*obj.features(data.x)*obj.features(data.x)'+inv(obj.Sigma);
                
                obj.theta = Sigma_inv^-1*(obj.Sigma^-1*obj.theta + 1/(obj.params.noiseCovariance)^2*obj.features(data.x)*data.y');

                obj.Sigma = Sigma_inv^-1;
            end
            
        end        
        
        function [y, var] = predict(obj, x)
            %PREDICT Predict function value
            %   Predict function value based on input and estimated
            %   parameters
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            y=obj.features(x)'*obj.theta;
            for i= 1:size(x,2)
                var(i,:)=obj.features(x(:,i))'*obj.Sigma*obj.features(x(:,i));
            end
        end
        
        function update_estimate(obj, data)
            %UPDATE_ESTIMATE Update estimate given new data
            % Update theta based on new inputs data.x and measurements data.y
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    if size(data.x,2)~=size(data.y,2)
                        error('Wrong data format!')
                    end
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            Sigma_inv = 1/(obj.params.noiseCovariance)^2*obj.features(data.x)*obj.features(data.x)'+inv(obj.Sigma);
                
            obj.theta = Sigma_inv\(obj.Sigma\obj.theta + 1/(obj.params.noiseCovariance)^2*obj.features(data.x)*data.y);
                
            obj.Sigma = inv(Sigma_inv);
            
        end
    end
end

