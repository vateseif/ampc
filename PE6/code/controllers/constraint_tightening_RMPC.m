%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2022, ETH Zurich, {adidier, jsieber}@ethz.ch
% 
% This code is only made available for students taking the advanced MPC class
% in the fall semester of 2022 (151-0371-00L) and is NOT to be distributed.
%
% Authors: Alexandre Didier, Jérôme Sieber
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef constraint_tightening_RMPC < Controller
    %ct-RMPC Construct an instance of this class
    %   Construct the robust constraint tightening MPC Class
    
    properties
        solver % solver name
        
        % since this controller uses feedback, we also store K
        K % feedback gain
        
        % since we need to compute the RPI set, we additionally store the
        % system information
        sys % System object
    end
    
    methods
        function obj = constraint_tightening_RMPC(sys, params, K, P, solver)
            %ct-RMPC Construct an instance of this class
            %   Construct the robust constraint tightening MPC Class and
            %   initialize solver
            
            %%% Parse inputs %%%
            switch nargin
                case 2
                    [K, P] = dlqr(sys.A, sys.B, params.Q, params.R);
                    K = -K;
                    %solver = 'quadprog';
                    % had to change the solver because matlab optimization
                    % toolbox doesn't work on my machine :(
                    solver = 'sedumi';
                case 3
                    error('Wrong number of inputs!')
                    
                case 4
                    solver = 'quadprog';
                    
                case 5
                                        
                otherwise 
                    error('Wrong number of inputs!')
            end
            
            %%%
            
            % call super class constructor
            obj@Controller(sys, params);
            
            % store system information
            obj.sys = sys;
            
            % initialize solver name
            obj.solver = solver;
            
            %%% Initialize the constraint tightening RMPC Controller
            % --- compute tightening ---
            obj.K = K;          
            F = obj.compute_robust_tightening(K);
            
            % --- compute terminal set ---
            X_f = obj.compute_MRPI(K);
            
            % look up horizon length
            N = obj.params.N;
            
            % look up state and input dimensions
            n = obj.sys.n;
            m = obj.sys.m;
            
            % look up system matrices
            A = obj.sys.A;
            B = obj.sys.B;
            
            % look up cost matrices
            Q = obj.params.Q;
            R = obj.params.R;
            
            % look up state, input, and disturbance sets
            X = obj.sys.X;
            U = obj.sys.U;
                        
            % --- define optimization variables ---
            z = sdpvar(n, N+1);
            z_0 = sdpvar(n, 1);
            v = sdpvar(m, N);
            
            % --- define objective ---
            objective = 0;
            for i = 1:N
                objective = objective + z(:,i)'*Q*z(:,i) + v(:,i)'*R*v(:,i);
            end
            objective = objective + z(:,N+1)'*P*z(:,N+1);
            
            % --- define constraints ---
            constraints = [z(:,1) == z_0];
            constraints = [constraints, z(:,1+1) == A*z(:,1) + B*v(:,1)];
            constraints = [constraints, X.A*z(:,1)<=X.b];% in first time step we have no tightening
            constraints = [constraints, U.A*v(:,1)<=U.b];
            for i = 1:N
                constraints = [constraints, z(:,i+1) == A*z(:,i) + B*v(:,i)];
                Z_i = X - F{i}; Z_i = Z_i.minHRep();
                constraints = [constraints, Z_i.A*z(:,i)<=Z_i.b];
                V_i = U - K*F{i}; V_i = V_i.minHRep();
                constraints = [constraints, V_i.A*v(:,i)<=V_i.b];
            end
            Z_f = X_f - F{N+1}; Z_f = Z_f.minHRep();
            constraints = [constraints, Z_f.A*z(:,N+1)<=Z_f.b];
            
            % --- setup Yalmip solver object ---
            obj.prob=optimizer(constraints, objective, sdpsettings('solver',solver), {z_0}, {v(:,1),v,z});    
        end
        
        function F = compute_robust_tightening(obj, K)
            %COMPUTE TIGHTENING Computes the disturbance reachable sets (DRS)
            %   and the corresponding tightenings.
            
            %%% Parse inputs %%%
            switch nargin
                case 1
                    K = -dlqr(obj.sys.A, obj.sys.B, obj.params.Q, obj.params.R);
                    
                case 2
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%
            
            % look up horizon length
            N = obj.params.N;
            
            % look up system matrices
            A = obj.sys.A;
            B = obj.sys.B;
            
            % disturbance set
            W = obj.sys.W;
            
            % compute disturbance reachable sets (DRS)
            F{1} = Polyhedron(); % initialize empty set
            for i = 1:N
                F_temp = F{i} + ((A+B*K)^(i-1))*W;
                F{i+1} = F_temp.minHRep(); % we need this to reduce the computational complexity
            end
        end
        
        function F = compute_MRPI(obj, K)
            %COMPUTE MRPI Computes maximal robust positive invariant set of
            %   the system.
            %%% Parse inputs %%%
            switch nargin
                case 2
                    
                otherwise 
                    error('Wrong number of inputs!')
            end
            %%%%%%%%%%%%%%%%%%%
            
            MRPI = ULTISystem('A', obj.sys.A + obj.sys.B*K, 'E', eye(obj.sys.n));
            MRPI.x.min = [-obj.sys.X.b(2); -obj.sys.X.b(4)];
            MRPI.x.max = [obj.sys.X.b(1); obj.sys.X.b(3)];
            MRPI.d.min = [-obj.sys.W.b(2); -obj.sys.W.b(4)];
            MRPI.d.max = [obj.sys.W.b(1); obj.sys.W.b(3)];

            disp("Computing MRPI set...");
            F = MRPI.invariantSet();
            disp("done!");
        end
    end
end

