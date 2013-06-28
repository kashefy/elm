function [ ] = display_prediction_stats(par_firingProbs, par_arr_label_id, par_figure_offset, par_figure_title)
    
    [nof_learners, nof_classes] = size(par_firingProbs);
    figure(par_figure_offset+0);
    nof_plot_rows = nof_learners;
    nof_plot_slots_per_row = 4;
    nof_plot_cols = 1;
    
    assert(nof_classes == length(par_arr_label_id));
    
    max_prob = max(par_firingProbs(:));

    for i = 1:nof_learners
        
        plot_slot_start = (i-1)*nof_plot_slots_per_row + 1;
        plot_slot_end = plot_slot_start + nof_plot_slots_per_row-1;
         
        subplot(nof_plot_slots_per_row*nof_plot_rows, nof_plot_cols, [plot_slot_start, plot_slot_end]);
         
        bar(par_firingProbs(i, :));
        if i==1
            title(['firing probs per learner', ' | ', par_figure_title], 'Interpreter', 'none')
        end
        ylabel(['p(z', num2str(i), '|c)']);
        axis tight;
        ylim([0 max_prob]);
        if i==nof_learners
            %xlabel(['class c, cEntr=', num2str(par_condEntropy(i))]);
            xlabel('class c');
            set(gca, 'XTickLabel', par_arr_label_id);
        else
            set(gca, 'XTick', []);
        end
    end
    
    set(gcf, 'PaperPositionMode', 'auto');
    set(gcf, 'units', 'normalized', 'outerposition', [0 0 1 1]);
    saveas(gcf, ['prediction_stats_hist_', par_figure_title, '.png']);
    
    
    %% 
    figure(par_figure_offset+2);
    
    subplot(2, 3, 1)
    p = par_firingProbs;
    p(p==0) = 1e-99;
    p = bsxfun(@rdivide, p, sum(p, 1));
    barh(1:nof_classes, -sum(p.*log2(p), 1));
    xlabel('cond. entropy(c_i)');
    ylabel('c_i');
    set(gca, 'YTickLabel', par_arr_label_id);
    axis tight
    set(gca, 'YDir', 'reverse');
    
    sub_plot = subplot(2, 3, 2);
    colormap('winter');
    imagesc(par_firingProbs');
    
    ylabel('c_i');
    set(gca, 'YTick', 1:nof_classes, 'YTickLabel', par_arr_label_id);
    xlabel('z_j');
    title(['firing probs per learner', ' | ', par_figure_title], 'Interpreter', 'none');
    
    pos_gca = get(sub_plot, 'position');
    handle_colorbar = colorbar('location', 'EastOutside');
    ylabel(handle_colorbar, 'p(z_j|c_i)');
    set(sub_plot, 'position', pos_gca);
    set(handle_colorbar, 'location', 'EastOutside');
    
    subplot(2, 3, 3);
    p = sort(par_firingProbs, 1);
    plot(p);
    
    ylabel('p(z_j|c_i)');
    set(gca, 'YTick', 1:nof_classes, 'YTickLabel', par_arr_label_id);
    xlabel('z_j');
    title(['distr. firing probs per learner', ' | ', par_figure_title], 'Interpreter', 'none');
    
    subplot(2, 3, 5)
    p = par_firingProbs;
    p(p==0) = 1e-99;
    p = bsxfun(@rdivide, p, sum(p, 2));
    bar(1:nof_learners, -sum(p.*log2(p), 2));
    ylabel('cond. entropy(z_j)');
    xlabel('z_j');
    axis tight
    ylim([0, -log2(1/nof_learners)]);
    
    % distance matrix
    distance = zeros(nof_classes);
    for i=1:nof_classes
        diff = bsxfun(@minus, par_firingProbs, par_firingProbs(:, i));
        distance_i = sqrt(sum(diff.^2, 1));
        distance(:, i) = distance_i;
    end
    sub_plot = subplot(2, 3, 4);
    imagesc(distance);
    title('distance matrix');
    xlabel('c_i');
    ylabel('c_j');
    set(gca, 'XTickLabel', par_arr_label_id, 'YTickLabel', par_arr_label_id);
    
    pos_gca = get(sub_plot, 'position');
    handle_colorbar = colorbar('location', 'EastOutside');
    ylabel(handle_colorbar, 'distance', 'FontSize', 8);
    set(sub_plot, 'position', pos_gca);
    set(handle_colorbar, 'location', 'EastOutside');
    
    set(gcf, 'PaperPositionMode', 'auto');
    set(gcf, 'units', 'normalized', 'outerposition', [0 0 1 1]);
    saveas(gcf, ['prediction_stats_heat_', par_figure_title, '.png']);

    %% Kullback–Leibler divergence
    figure(par_figure_offset+3)
    kl_divergence = zeros(nof_classes);
    p = par_firingProbs;
    p = bsxfun(@rdivide, p, sum(p, 1));
    p(p==0) = NaN;
    p_log = log(p);
    for i=1:nof_classes
        assert(abs(nansum(p(:, i))-1.0) < 1e-5); % distributions must sum to 1
        diff = bsxfun(@minus, p_log(:, i), p_log);
        prod = bsxfun(@times, diff, p_log(:, i));
        divergence_i = nansum(prod, 1);
        kl_divergence(:, i) = divergence_i;
    end
    sub_plot = subplot(1, 1, 1);
    kl_divergence = kl_divergence + kl_divergence'; % symmetrised divergence
    imagesc(kl_divergence);
    title('D_K_L(c_i||c_j)+D_K_L(c_j||c_i)')
    xlabel('c_i');
    ylabel('c_j');
    set(gca, 'XTickLabel', par_arr_label_id, 'YTickLabel', par_arr_label_id);
    
    axis image
    
    handle_colorbar = colorbar('location', 'EastOutside');
    ylabel(handle_colorbar, 'KL divergence');
    
    set(gcf, 'PaperPositionMode', 'auto');
    set(gcf, 'units', 'normalized', 'outerposition', [0 0 1 1]);
    saveas(gcf, ['prediction_stats_kl_', par_figure_title, '.png']);

    
    
end