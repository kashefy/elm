function [] = display_masks(par_masks_learners, par_figure_id, par_masks_singles)

    nof_masks = length(par_masks_learners);
    
    if nargin > 2 % have mask samples?

        nof_plot_rows = nof_masks;
        nof_plot_cols = 1;
        
        [~, nofSingleMasks] = size(par_masks_singles);
        nof_plot_cols = nof_plot_cols + nofSingleMasks;
    else
        nof_plot_rows = 1;
        nof_plot_cols = nof_masks;
    end
    
    nof_plot_slots_per_col = 4;
    %clims = [min(nodes(:)), max(nodes(:))];
    clims = [0, 1];
    figure(par_figure_id)
    plotIndex = 1;
    for i = 1:nof_masks
        
        plot_slot_start = (plotIndex-1)*nof_plot_slots_per_col + 1;
        plot_slot_end = plot_slot_start + nof_plot_slots_per_col-1;
        subplot(nof_plot_rows, nof_plot_slots_per_col*nof_plot_cols, [plot_slot_start, plot_slot_end])
    
        %subplot(nof_plot_rows, nof_plot_cols, plotIndex);
        imagesc(par_masks_learners{i}, clims);
        colormap(gray);
        axis image
        if i==1
            title('mask aggregates');
        end
        if i < nof_masks
            set(gca, 'XTick', []);
        end
        h = ylabel(['m', num2str(i-1)]);
        ylabel(h, 'FontSize', 5);

        plotIndex = plotIndex+1;
        if nargin > 2 % have mask samples?
            
            for mi = 1:nofSingleMasks
                
                if ~isempty(par_masks_singles{i,mi})
                    
                    plot_slot_start = (plotIndex-1)*nof_plot_slots_per_col + 1;
                    plot_slot_end = plot_slot_start + nof_plot_slots_per_col-1;
                    subplot(nof_plot_rows, nof_plot_slots_per_col*nof_plot_cols, [plot_slot_start, plot_slot_end])

                    %subplot(nof_plot_rows, nof_plot_cols, plotIndex);
                    imagesc(par_masks_singles{i,mi},[0, 1]);
                    
                    colormap(gray);
                    set(gca, 'YTick', []);
                    if i < nof_masks
                        set(gca,'XTick', []);
                    end
                    axis image
                    if(i==1 && mi == ceil(nofSingleMasks/2))
                        title('mask samples');
                    end
                end
                plotIndex = plotIndex+1;
            end
        end
    end
    
    
end