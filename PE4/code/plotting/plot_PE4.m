function plot_PE4(figNr, nrSteps, X_tight, U_tight, X, U, P, p_tilde, params)
%PLOT PE 4 Summary of this function goes here
%   Detailed explanation goes here

    %%% Parse input arguments %%%
    switch nargin
        case 4
            X = [];
            U = [];
            P = [];
            p_tilde = [];
            params = {};

        case 5
            U = [];
            P = [];
            p_tilde = [];
            params = {};
        
        case 6
            P = [];
            p_tilde = [];
            params = {};
            
        case 7
            p_tilde = [];
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
    if any(X_tight.b <= 0) % TODO: maybe replace this with empty set check as this can trigger when the origin is not contained in the set
        error("State tightening is larger than state constraints. Hence, the SMPC problem will always be infeasible!")
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
    if ~isempty(P) && ~isempty(p_tilde)
        hold on
        x=sdpvar(2,1);
        if ~iscell(P)
        p3 = YSet(x,[x'*P*(pi/180)^2*x<=p_tilde]).plot('Color','b','alpha',0.4);
        else 
            for i=1:length(P)
                if i==length(P)
                    p3 = YSet(x,[x'*P{length(P)+1-i}*(pi/180)^2*x<=p_tilde]).plot('Color','b','alpha',0.3/(length(P)+1-i),'EdgeAlpha',0.3);
                else
                    YSet(x,[x'*P{length(P)+1-i}*(pi/180)^2*x<=p_tilde]).plot('Color','b','alpha',0.3/(length(P)+1-i),'EdgeAlpha',0.3);
                end
               
            end
        end
    else
        p3 = [];
    end
    xlabel('position [deg]')
    ylabel('velocity [deg/s]')
    legend([p1,p2,p3],{'tightened constraints', 'original constraints', 'PRS set'}, 'Position',[0.35 0.82 0.1 0.1])
    
    subplot(1,2,2)
    if any(U_tight.b <= 0) % TODO: maybe replace with empty set check as above
        error("Input tightening is larger than input constraints. Hence, the robust nonlinear MPC problem will always be infeasible!")
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

