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

classdef RMPC < Controller
    %RMPC Robust MPC Controller Class
    %   Construct and solve robust linear MPC Problem
    
    properties
        solver % solver name
        
        % since this controller uses feedback, we also store K
        K % feedback gain
        
        sys %System description
    end
    
    methods
        function obj = RMPC(sys, params, rho, solver)
            %   RMPC Construct an instance of this class
            %   Construct the robust MPC Class and
            %   initialize solver
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    rho = 0.9;
                    solver = 'sedumi';
                    
                case 3
                    solver = 'sedumi';
                    
                case 4
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            
            %%%
            
            % call super class constructor
            obj@Controller(sys, params);
            
            % Internal system description
            obj.sys = sys;
            
            % initialize solver name
            obj.solver = solver;
            
            %%% Initialize the RMPC Controller
            
            % --- define optimization variables ---
            z = sdpvar(n, N+1); % Nominal prediction states
            x_k = sdpvar(n, 1); % Current true state, which is the input to the obj.prob optimizer object
            v = sdpvar(m, N);   % Nominal prediction inputs
            
            %%% TODO %%%
            % Compute the required tightenings given rho
            % You have to define obj.K!
            % Define the robust MPC problem constraints and objective.
            
            % --- start inserting here ---            
            

            % --- stop inserting here ---
            %%%
            
            % --- setup Yalmip solver object ---
            obj.prob=optimizer(constraints, objective, sdpsettings('solver',solver), {x_k}, {v(:,1),z(:,1)});    
        end
        
        function [x_tight, u_tight, P, K, delta] = compute_tightening(obj, rho)
            %COMPUTE TIGHTENING Computes an RPI set and the corresponding
            %   tightening, which minimizes the constraint tightening.
            
            %%% Parse inputs %%%
            switch nargin
                case 1
                    rho = 0.9;
                    
                case 2
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%
                       
             
            %--- setup & solve offline optimization problem ---
            
            %%% TODO %%%
            % Define the SDP for computing a contractive ellipsoid
            % Compute the sublevel set delta which renders the ellipsoid
            % RPI as well as the corresponding state and input constraint
            % tightenings. 
            %
            % Make sure to define every output of the function!
            %
            % We recommend weighing the state tightenings in the objective
            % function by a factor of 50, i.e., 50*sum(c_{x,j}^2) in order
            % to obtain good results. However, any RPI set which you
            % compute with the provided constraints in 1a) and which 
            % results in a recursively feasible RMPC scheme, as well as 
            % feasible for the provided initial condition will be accepted.
            % --- start inserting here ---            
            
            options = sdpsettings('solver','sedumi', 'verbose', 0);
            % Only uncomment if you have Mosek installed, it will result in
            % a nice speedup
            %options = sdpsettings('solver','mosek', 'verbose', 0);
            
            % --- stop inserting here ---
            %%%
            
        end
    end
end

