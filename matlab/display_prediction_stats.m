function [ ] = display_prediction_stats(par_firingProbs, par_condEntropy, par_figure_offset, par_figure_title)

    [nofLearners, nof_classes] = size(par_firingProbs);
    figure(par_figure_offset+0)   
    nof_plot_rows = nofLearners;
    nof_plot_slots_per_row = 4;
    nof_plot_cols = 1;
    
    max_prob = max(par_firingProbs(:));

    for i = 1:nofLearners
        
        plot_slot_start = (i-1)*nof_plot_slots_per_row + 1;
        plot_slot_end = plot_slot_start + nof_plot_slots_per_row-1;
         
        subplot(nof_plot_slots_per_row*nof_plot_rows, nof_plot_cols, [plot_slot_start, plot_slot_end])
         
        bar(par_firingProbs(i, :));
        if i==1
            title(['firing probs per learner', ' | ', par_figure_title], 'Interpreter', 'none')
        end
        ylabel(['p(z', num2str(i), '|c)']);
        axis tight;
        ylim([0 max_prob]);
        if i==nofLearners
            %xlabel(['class c, cEntr=', num2str(par_condEntropy(i))]);
            xlabel('class c');
        else
            set(gca, 'XTick', []);
        end
    end
    
    set(gcf, 'PaperPositionMode', 'auto');
    set(gcf, 'units', 'normalized', 'outerposition', [0 0 1 1]);
    saveas(gcf, ['prediction_stats_hist_', par_figure_title, '.png']);
    
    
    %% 
    figure(par_figure_offset+2)
    
    subplot(2, 2, 1)
    barh(1:nof_classes, -sum(par_firingProbs.*log2(par_firingProbs), 1))
    xlabel('cond. entropy(c_i)');
    ylabel('c_i');
    axis tight
    %ylim([0, -log2(1/nof_classes)])
    
    sub_plot = subplot(2, 2, 2);
    imagesc(par_firingProbs');
    ylabel('c_i');
    xlabel('z_j');
    title(['firing probs per learner', ' | ', par_figure_title], 'Interpreter', 'none')
    
    pos_gca = get(sub_plot, 'position');
    handle_colorbar = colorbar('location', 'EastOutside');
    ylabel(handle_colorbar, 'p(z_j|c_i)');
    set(sub_plot, 'position', pos_gca);
    set(handle_colorbar, 'location', 'EastOutside');
    
    subplot(2, 2, 4)
    bar(1:nofLearners, -sum(par_firingProbs.*log2(par_firingProbs), 2))
    ylabel('connd. entropy(z_j)');
    xlabel('z_j');
    axis tight
    ylim([0, -log2(1/nofLearners)])
    
    distance = zeros(nof_classes);
    for i=1:nof_classes
        diff = bsxfun(@minus, par_firingProbs, par_firingProbs(:, i));
        distance_i = sqrt(sum(diff.^2, 1));
        distance(:, i) = distance_i;
    end
    sub_plot = subplot(2, 2, 3);
    imagesc(distance);
    title('distance matrix(c)')
    xlabel('c_i');
    ylabel('c_j');
    
    pos_gca = get(sub_plot, 'position');
    handle_colorbar = colorbar('location', 'EastOutside');
    ylabel(handle_colorbar, 'distance', 'FontSize', 8);
    set(sub_plot, 'position', pos_gca);
    set(handle_colorbar, 'location', 'EastOutside');
    
    set(gcf, 'PaperPositionMode', 'auto');
    set(gcf, 'units', 'normalized', 'outerposition', [0 0 1 1]);
    saveas(gcf, ['prediction_stats_heat_', par_figure_title, '.png']);

    
end