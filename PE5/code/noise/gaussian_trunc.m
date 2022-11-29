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

function [w] = gaussian_trunc(mean, covariance, H_w, h_w)
%GAUSSIAN Creates a truncated Gaussian distubance
%   Computes Gaussian disturbance based on noise mean and covariance in the
%   set H_w w <= h_w

%%% Parse inputs %%%
switch nargin
    case 4
        
    otherwise
        error('Wrong number of inputs!')
end
%%%%%%%%%%%%%%%%%%%

w = mean + chol(covariance,'L')*randn(size(mean));
while any(H_w*w>h_w)
    w = mean + chol(covariance,'L')*randn(size(mean));
end
end

