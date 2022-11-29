function plot_rec10(figNr, nrSteps, X_tight, U_tight, X, U, P, params)
%PLOT REC 10 Summary of this function goes here
%   Detailed explanation goes here

    %%% Parse input arguments %%%
    switch nargin
        case 4
            X = [];
            U = [];
            P = [];
            params = {};

        case 5
            U = [];
            P = [];
            params = {};
        
        case 6
            P = [];
            params = {};
            
        case 7
            params = {};
           
        case 8

        otherwise
            error('Wrong number of inputs!')
    end
    %%%%%%%%%%%%%%%%%%%
    
    figure(figNr)
    subplot(1,2,1)
    if ~iscell(X_tight)
        X_tight = X_tight*(180/pi);
        p1 = X_tight.plot('wire', true, 'linestyle', '--', 'linewidth', 1.5);
    else
        N = length(X_tight);
        for i=1:N
            hold on;
            if i==1
                X_temp = X_tight{i}*(180/pi);
                p3 = X_temp.plot('wire', true, 'linestyle', '--', 'linewidth', 0.2);
            else
                X_temp = X_tight{i}*(180/pi);
                X_temp.plot('wire', true, 'linestyle', '--', 'linewidth', 0.2);
            end
        end
    end
    if ~isempty(X)
        hold on;
        X = X*(180/pi);
        p2 = X.plot('wire', true, 'linestyle', '-', 'linewidth', 2);
    else
        p2 = [];
    end
    if ~isempty(P)
        hold on
        x=sdpvar(2,1);
        if ~iscell(P)
            p3 = P.plot('Color','b','alpha',0.4);
        else
            N = length(P);
            for i=1:N
                hold on;
                if i==1
                    P_temp = P{N+1-i}*(180/pi);
                    p3 = P_temp.plot('Color','b','alpha',1/N+0.1,'edgealpha',1);
                else
                    P_temp = P{N+1-i}*(180/pi);
                    P_temp.plot('Color','b','alpha',1/N+0.05,'edgealpha',0.5);
                end
            end
        end
    else
        p3 = [];
    end
    xlabel('position [deg]')
    ylabel('velocity [deg/s]')
    %legend([p1,p2,p3],{'tightened constraints', 'original constraints', 'RPI set'}, 'Position',[0.35 0.82 0.1 0.1])
    
    subplot(1,2,2)
    if ~iscell(U_tight)
        plot(linspace(-1, nrSteps+1,2),max(U_tight.V)*ones(1,2), 'color', 'k', 'linewidth',1.5,'linestyle','--')
        hold on;
        plot(linspace(-1, nrSteps+1,2),min(U_tight.V)*ones(1,2), 'color', 'k', 'linewidth',1.5,'linestyle','--')
    else
        N = length(U_tight);
        for i=1:N
            hold on;
            plot(linspace(-1, nrSteps+1,2),max(U_tight{i}.V)*ones(1,2), 'color', 'k', 'linewidth',0.2,'linestyle','--')
            hold on;
            plot(linspace(-1, nrSteps+1,2),min(U_tight{i}.V)*ones(1,2), 'color', 'k', 'linewidth',0.2,'linestyle','--')
        end
    end
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

