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

classdef LinearAffineSystem < System
    %SYSTEM System class implementing a segway system
    %   The System class defines the interfaces, which will be used by the
    %   controller and parameter classes implemented in the recitations.
    properties
        A_delta %dynamic matrices of affine dynamics parameterization; A = A_delta{0} + \sum A_delta{i} * theta(i)
        B_delta %dynamic matrices of affine dynamics parameterization; B = B_delta{0} + \sum B_delta{i} * theta(i)
        A %dynamic system matrix
        B %input matrix 
        C %state output matrix 
        D %input feedthrough matrix
        f %update function
        h %output function
        theta %uncertain parameter estimate
        p %dimension of uncertain parameter
        Omega %uncertain parameter set
    end
    methods
        function obj = LinearAffineSystem(params)
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
            
            % define parameter set
            if ~isempty(obj.params.A_theta) && ~isempty(obj.params.b_theta)
                obj.Omega = Polyhedron(obj.params.A_theta, obj.params.b_theta);
            end
            
            % Uncertain Parameter
            obj.p = obj.params.p;
            obj.theta = obj.params.theta;
            
            % define dynamic uncertainty in A & B
            obj.A_delta = obj.params.A_delta;
            obj.B_delta = obj.params.B_delta;
            
            % define system matrices
            obj.C = obj.params.C;
            obj.D = obj.params.D;
            
            obj.A=obj.A_delta{1};
            for i=2:length(obj.A_delta)
                obj.A = obj.A+obj.A_delta{i}*obj.theta(i-1);
            end
            obj.B=obj.B_delta{1};
            for i=2:length(obj.B_delta)
                obj.B = obj.B+obj.B_delta{i}*theta(length(obj.A_delta)-1+i-1);
            end
            % define update law
            obj.f = @(x,u) obj.A*x + obj.B*u + obj.generateNoise(obj.noiseArgs{:});
            obj.h = @(x,u) obj.C*x + obj.D*u;
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
            
            % look up variables
            A_x = obj.params.A_x;
            b_x = obj.params.b_x;
            A_u = obj.params.A_u;
            b_u = obj.params.b_u;
            A_w = obj.params.A_w;
            b_w = obj.params.b_w;

            Omega_i = Polyhedron([A_x; A_u*K],[b_x; b_u]);

            while true
                % Compute support function h_W(F) max_(w in W) F*w
                n_halfspaces = size(Omega_i.A,1);
                h_W_F = zeros(n_halfspaces,1);
                for i=1:n_halfspaces
                    [~,h_W_F(i)]=linprog(Omega_i.A(i,:)', A_w, b_w, [],[],[],[],optimoptions('linprog','Display','off'));
                end
                % compute Pre-set of Omega
                Pre_W_Omega = Polyhedron(Omega_i.A*(obj.A + obj.B*K), Omega_i.b-h_W_F);
                % intersect Pre-set of Omega and Omega
                Omega_i1 = Polyhedron([Omega_i.A; Pre_W_Omega.A], [Omega_i.b; Pre_W_Omega.b]);
                Omega_i1.minHRep();
                % terminate if Omega does not change anymore
                if Omega_i1 == Omega_i
                    break
                end
                Omega_i = Omega_i1;
            end
            F = Omega_i;
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
            
            % define parameter set
            if ~isempty(obj.params.A_theta) && ~isempty(obj.params.b_theta)
                obj.Omega = Polyhedron(obj.params.A_theta, obj.params.b_theta);
            end
            
            % Uncertain Parameter
            obj.theta = obj.params.theta;
            
            % define dynamic uncertainty in A & B
            obj.A_delta = obj.params.A_delta;
            obj.B_delta = obj.params.B_delta;
            
            
            % define system matrices
            obj.C = obj.params.C;
            obj.D = obj.params.D;
            
            obj.A=obj.A_delta{1};
            for i=2:length(obj.A_delta)
                obj.A = obj.A+obj.A_delta{i}*obj.theta(i-1);
            end
            obj.B=obj.B_delta{1};
            for i=2:length(obj.B_delta)
                obj.B = obj.B+obj.B_delta{i}*obj.theta(length(obj.A_delta)-1+i-1);
            end
            % define update law
            obj.f = @(x,u) obj.A*x + obj.B*u + obj.generateNoise(obj.noiseArgs{:});
            obj.h = @(x,u) obj.C*x + obj.D*u;
        end
    end
end