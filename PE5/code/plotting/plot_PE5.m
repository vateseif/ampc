function plot_PE5(figNr, Omega_0, Delta, Omega_1, params)
%PLOT REC 11 Summary of this function goes here
%   Detailed explanation goes here

    %%% Parse input arguments %%%
    switch nargin
        case 4
            params = {};
            
        case 5
            
        otherwise
            error('Wrong number of inputs!')
    end
    %%%%%%%%%%%%%%%%%%%
    
    figure(figNr)
    hold on
    Omega_0.plot('alpha', 0.3, 'color', 'red');
    Delta = Polyhedron([Delta.A;1,0;-1,0],[Delta.b;12;-7]);
    Delta.plot('alpha', 0.3, 'color', 'blue');
    Omega_1.plot('alpha', 0.8,'color', 'k');
    axis([8 11.5 6.5 9.5])
    hold off
    if ~isempty(params)
        set(gcf,'position',[100,100,params.width,params.height],'color','white')
    end
end

