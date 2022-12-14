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

classdef KF < Parameter_Estimator
    %KF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        theta
        Sigma
        features
    end
    
    methods
        function obj = KF(sys,  prior_mean, prior_variance,  features)
            %KF Construct an instance of this class
            %   Construct Kalman Filter Class. The second
            %   input is a struct with data.x an nxN input data matrix and
            %   data.y an mxN output data matrix of N data points
            
            %%% Parse inputs %%%
            switch nargin
                case 3
                    features = @(x) x;
                    
                case 4
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            obj@Parameter_Estimator(sys);
            
            obj.theta = prior_mean;
            obj.Sigma = prior_variance;
            obj.features = features;
        end
        
        function [theta, Sigma] = prediction_update(obj)
            %PREDICT Predict function value
            %   Predict function value based on input and estimated
            %   parameters
            
            %%% Parse inputs %%%
            switch nargin
                case 1
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            %%% TODO
            % Implement prediction update. Use the parameter dynamics to
            % update the expected value in theta and its variance in Sigma.
            % The parameter noise covariance is provided in Sigma_w_theta
            % --------- Start Modifying Code Here -----------
            Sigma_w_theta = obj.params.paramNoise; % Parameter noise covariance Sigma^{w_\theta}
            
            % Compute the prediction update \theta_{p,k} as a function of the previous estimate \hat{\theta}_{k-1}=obj.theta
            theta = obj.theta;  
            % Compute the prediction update \Sigma^\theta_{p,k} as a function of the previous estimate \Sigma^\theta_{k-1}=obj.Sigma
            Sigma = obj.Sigma + Sigma_w_theta; 

            % --------- Stop Modifying Code Here -----------

        end
        
        function update_estimate(obj, data)
            %UPDATE_ESTIMATE Update estimate given new data
            % Update theta based on new inputs data.x and measurements data.y
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    
                otherwise
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            %%% TODO: 
            % Implement estimate update. Given the new data point, 
            % 
            % --------- Start Modifying Code Here -----------
            
            % Perform the prediction update
            [theta_p, Sigma_p] = obj.prediction_update();
            
            %%% TODO
            % Implement the measurement update. Use the equations provided in 
            % the recitation to update the parameters given .
            % The process noise (disturbance) covariance is provided in 
            % Sigma_w and the feature vector \Phi(x(k-1),u(k-1)) is provided in
            % Phi. The state measurement x(k) is provided in x_k.
            % --------- Start Modifying Code Here -----------
            
            x_k = data.y; % State measurement x(k)
            
            Sigma_w = obj.params.noiseCovariance; % Disturbance covariance matrix
            
            Phi = obj.features(data.x); % Feature vector given new data point \Phi(x(k-1),u(k-1)) 
            
            K = Sigma_p * Phi' / (Phi * Sigma_p * Phi' + Sigma_w);
            obj.Sigma = (eye(size(Sigma_p)) - K * Phi) * Sigma_p;
            obj.theta = theta_p + K * (x_k - Phi * theta_p);
            
            % --------- Stop Modifying Code Here -----------
            
            %%%
        end
    end
end

