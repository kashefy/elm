function [] = display_response_and_label(par_filepath_label, par_filepath_response, par_figure_offset, par_filepath_response_layer2)
                                        
    figure(par_figure_offset);
    if nargin > 3
        nof_plot_rows = 3;
    else
        nof_plot_rows = 2;
    end
    
    labels = dlmread(par_filepath_label);
    subplot(nof_plot_rows, 1, 1)
    plot(labels, '.')
    title('labels');
    ylabel('label id');
    
    response = dlmread(par_filepath_response);
    subplot(nof_plot_rows, 1, 2)
    plot(response, '.')
    title('response');
    ylabel('learner index');
    
    if nargin > 3
        response2 = dlmread(par_filepath_response_layer2);
        subplot(nof_plot_rows, 1, 3)
        plot(response2, '.')
        title('response layer(2)');
        ylabel('learner index');
        xlabel('t')
    else
        xlabel('t')
    end
    
    suptitle('labels and responses');
end