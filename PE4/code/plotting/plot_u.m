function plot_u(figNr, u, U, params)
%PLOT_U Summary of this function goes here
%   Detailed explanation goes here
    
    %%% Parse input arguments %%%
    switch nargin
        case 2
            U = [];
            params = {};

        case 3
            params = {};

        case 4

        otherwise
            error('Wrong number of inputs!')
    end
    %%%%%%%%%%%%%%%%%%%
    
    nrSteps = size(u,1);
    figure(figNr)
    plot(squeeze(u),'color',[params.color, params.alpha],'linewidth',params.lw)
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
    set(gcf,'position',[100,100,params.width,params.height],'color','white')
end

