function plot_PE6(figNr, Sigma_theta, theta_mean, theta_true, X2_n, Omega, params, plot_surf)
%PLOT REC 11 Summary of this function goes here
%   Detailed explanation goes here

    %%% Parse input arguments %%%
    switch nargin
        case 5
            Omega = [];
            params = {};
            plot_surf = true;
            
        case 6
            params = {};
            plot_surf = true;
            
        case 7
            plot_surf = true;
            
        case 8
            
        otherwise
            error('Wrong number of inputs!')
    end
    %%%%%%%%%%%%%%%%%%%
    
    [x_plot, y_plot] = meshgrid(4.2:0.02:5.8, 3.2:0.02:4.8);
    figure(figNr)
    hold on
    Lambda = inv(Sigma_theta);
    z_plot = exp(-0.5*((x_plot-theta_mean(1)).^2*Lambda(1,1)+(x_plot-theta_mean(1)).*(y_plot-theta_mean(2)).*Lambda(1,2)+(x_plot-theta_mean(1)).*(y_plot-theta_mean(2)).*Lambda(2,1)+(y_plot-theta_mean(2)).^2*Lambda(2,2)));
    z_plot_max=max(max(z_plot));
    if plot_surf
        p1=surf(x_plot, y_plot, z_plot/z_plot_max - 1,'FaceColor','interp','FaceAlpha',0.8,'EdgeColor','none');
    else
        p1 = [];
    end
    x=sdpvar(2,1);
    p2 = YSet(x,[(x - theta_mean)'*Lambda*(x - theta_mean)<=X2_n]).plot('wire', true, 'linestyle', '-', 'linewidth', 2, 'EdgeColor', 'k');
    p4 = scatter(theta_true(1), theta_true(2), 'ok');
    if ~isempty(Omega)
        Omega_temp = Omega+theta_mean;
        p3 = Omega_temp.plot('wire', true, 'linestyle', '-', 'linewidth', 2, 'EdgeColor', 'red');
    else
        p3 = [];
    end
    xlabel('\theta_1')
    ylabel('\theta_2')
    zlabel('Unnormalised Probability')
    if ~isempty(p3) && plot_surf
        if isempty(params.posterior)
           legend([p1,p2,p3,p4],{'Prior distribution', 'Confidence region', 'Overapproximation', 'True theta'},'Location','ne');
        else 
           legend([p1,p2,p3,p4],{'Posterior distribution', 'Confidence region', 'Overapproximation', 'True theta'},'Location','ne'); 
        end
    else
        if isempty(p3)
            if isempty(params.posterior)
                legend([p1,p2,p4],{'Prior distribution', 'Confidence region', 'True theta'},'Location','ne');
            else 
                legend([p1,p2,p4],{'Posterior distribution', 'Confidence region', 'True theta'},'Location','ne');
            end
        else
            legend([p2,p3,p4],{'Confidence region', 'Overapproximation', 'True theta'},'Location','ne');
        end
    end
    xlim([4.2, 5.8])
    ylim([3.2, 4.8])
    hold off
    if ~isempty(params)
        set(gcf,'position',[100,100,params.width,2*params.height],'color','white')
    end
    if ~isempty(params.title)
        title(params.title)
    else    
        title('Confidence Region')
    end
end

