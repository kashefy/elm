function [] = display_response_and_label(par_filepath_label, par_filepath_response, par_figure_offset, par_filepath_response_layerZ, par_filepath_membrane_pot_layerZ)
                                        
    figure(par_figure_offset);
    if nargin > 3
        nof_plot_rows = 5;
    else
        nof_plot_rows = 2;
    end
    
    labels = dlmread(par_filepath_label);
    subplot(nof_plot_rows, 1, 1)
    plot(labels, '.')
    title('labels');
    ylabel('label id');
    axis tight
    
    response = dlmread(par_filepath_response);
    subplot(nof_plot_rows, 1, 2)
    plot(response, '.')
    title('response');
    ylabel('learner index');
    set(gca, 'YDir', 'Reverse');
    axis tight
    
    if nargin > 3
        response_layerZ = dlmread(par_filepath_response_layerZ);
        subplot(nof_plot_rows, 1, 3)
        plot(response_layerZ, '.')
        title('response layerZ');
        ylabel('learner index');
        xlabel('t')
        set(gca, 'YDir', 'Reverse');
        axis tight
        
        membrane_pot_layerZ = dlmread(par_filepath_membrane_pot_layerZ, ',');
        subplot(nof_plot_rows, 1, 4)
        plot(membrane_pot_layerZ)
        title('u layerZ', 'LineWidth', 10);
        ylabel('mem. pot. u');
        %xlabel('t')
        %legend('show', 'Location', 'EastOutside');
        axis tight
        
        subplot(nof_plot_rows, 1, 5)
        imagesc(membrane_pot_layerZ')
        %title('u layerZ', 'LineWidth', 10);
        ylabel('u');
        xlabel('t')
        %legend('show', 'Location', 'EastOutside');
        axis tight
    else
        xlabel('t')
    end
    
    suptitle('labels and responses');
    
    set(gcf, 'PaperPositionMode', 'auto');
    set(gcf, 'units', 'normalized', 'outerposition', [0 0 1 1]);
    saveas(gcf, 'response_n_label.png');
    
end