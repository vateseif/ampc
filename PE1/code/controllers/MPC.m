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

classdef MPC < Controller
    %MPC Controller Class
    %   Construct and solve nominal MPC Problem
    
    properties
        % no additional class properties needed
    end
    
    methods
        function obj = MPC(sys, params, solver)
            %MPC Construct an instance of the MPC class
            %   initialize MPC optimization problem and define prob
            %   property
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    solver = '';
                    
                case 3
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%
            
            % call parent class constructor
            obj@Controller(sys, params);
            
            %%% --- Initialize MPC Controller ---
            % define optimization variables
            x = sdpvar(size(sys.A,1), obj.params.N+1);
            x_0 = sdpvar(size(sys.A,1),1);
            u = sdpvar(size(sys.B,2), obj.params.N);
            
            % define objective
            objective = 0;
            for i = 1:obj.params.N
                objective = objective + x(:,i)'*obj.params.Q*x(:,i) + u(:,i)'*obj.params.R*u(:,i);
            end
            % terminal constraint is the origin, therefore the terminal
            % cost is zero.
            
            % define constraints
            constraints = [x(:,1)==x_0];
            for i = 1:obj.params.N
                constraints = [constraints, x(:,i+1) == sys.A*x(:,i)+sys.B*u(:,i)];
                constraints = [constraints, sys.X.A*x(:,i)<=sys.X.b];
                constraints = [constraints, sys.U.A*u(:,i)<=sys.U.b];
            end
            constraints = [constraints, x(:,obj.params.N+1)==0];
            
            % setup Yalmip solver object
            obj.prob = optimizer(constraints, objective, sdpsettings('solver',solver), {x_0}, {u(:,1)});
        end
    end
end

