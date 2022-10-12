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

function plot_cost(figNr, cost, params, label_legend, label_title)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    %%% Parse input arguments %%%
    switch nargin
        case 2
            params = {};
            label_legend = '';
            label_title = '';
            
        case 3
            label_legend = '';
            label_title = '';
        
        case 4
            label_title = '';
            
        case 5

        otherwise
            error('Wrong number of inputs!')
    end
    %%%%%%%%%%%%%%%%%%%
    
    nrSteps = size(cost,1);
    nrTraj = size(cost,2);
    
    figure(figNr)
    hold on
    colormap winter;
    plot(cost(:,:),'color',[params.color,params.alpha],'linewidth',params.lw);
    if ~isempty(label_legend)
        legend(p, label_legend);
    end
    if ~isempty(label_title)
        title(label_title);
    end
	xlabel('time')
    ylabel('cost')
    xlim([0,nrSteps+2])
    grid()
    set(gcf,'position',[100,100,params.width,2*params.height],'color','white')
    hold off
end

