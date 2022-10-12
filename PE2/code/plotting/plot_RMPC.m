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

function plot_RMPC(figNr, nrSteps, X_tight, U_tight, X, U, P, delta, params)
%PLOT REC 3 Summary of this function goes here
%   Detailed explanation goes here

    %%% Parse input arguments %%%
    switch nargin
        case 4
            X = [];
            U = [];
            P = [];
            delta = [];
            params = {};

        case 5
            U = [];
            P = [];
            delta = [];
            params = {};
        
        case 6
            P = [];
            delta = [];
            params = {};
            
        case 7
            delta = [];
            params = {};
           
        case 8
            params = {};
            
        case 9

        otherwise
            error('Wrong number of inputs!')
    end
    %%%%%%%%%%%%%%%%%%%
    
    figure(figNr)
    subplot(1,2,1)
    if any(X_tight.b <= 0)
        error("State tightening is larger than state constraints. Hence, the robust MPC problem will always be infeasible!")
    end
    X_tight = X_tight*(180/pi);
    p1 = X_tight.plot('wire', true, 'linestyle', '--', 'linewidth', 1.5);
    if ~isempty(X)
        hold on
        X = X*(180/pi);
        p2 = X.plot('wire', true, 'linestyle', '-', 'linewidth', 2);
    else
        p2 = [];
    end
    if ~isempty(P) && ~isempty(delta)
        hold on
        x=sdpvar(2,1);
        p3 = YSet(x,[x'*P*(pi/180)^2*x<=delta^2]).plot('Color','b','alpha',0.4);
    else
        p3 = [];
    end
    xlabel('position [deg]')
    ylabel('velocity [deg/s]')
    legend([p1,p2,p3],{'tightened constraints', 'original constraints', 'RPI set'}, 'Position',[0.35 0.82 0.1 0.1])
    
    subplot(1,2,2)
    if any(U_tight.b <= 0)
        error("Input tightening is larger than input constraints. Hence, the robust MPC problem will always be infeasible!")
    end
    plot(linspace(-1, nrSteps+1,2),max(U_tight.V)*ones(1,2), 'color', 'k', 'linewidth',1.5,'linestyle','--')
    hold on;
    plot(linspace(-1, nrSteps+1,2),min(U_tight.V)*ones(1,2), 'color', 'k', 'linewidth',1.5,'linestyle','--')
    if ~isempty(U)
        hold on;
        plot(linspace(-1, nrSteps+1,2),max(U.V)*ones(1,2), 'color', 'k','linewidth',2)
        hold on;
        plot(linspace(-1, nrSteps+1,2),min(U.V)*ones(1,2), 'color', 'k','linewidth',2)
    end
    xlabel('time')
    ylabel('input')
    xlim([0,nrSteps+1])
    grid()
    if ~isempty(params)
        set(gcf,'position',[100,100,params.width,params.height],'color','white')
    end
end

