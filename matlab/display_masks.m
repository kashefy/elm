function [] = display_masks(par_masks_learners, par_figure_id, par_masks_singles)

    nofMasks = length(par_masks_learners);
    nofPlotRows = nofMasks;
    nofPlotCols = 1;
    if nargin > 2 % have mask samples?
        [~, nofSingleMasks] = size(par_masks_singles);
        nofPlotCols = nofPlotCols+nofSingleMasks;
    end
    %clims = [min(nodes(:)), max(nodes(:))];
    clims = [0, 1];
    figure(par_figure_id)
    plotIndex = 1;
    for i = 1:nofMasks
        
        subplot(nofPlotRows, nofPlotCols, plotIndex);
        imagesc(par_masks_learners{i}, clims);
        colormap(gray);
        axis image
        if i==1
            title('mask aggregates');
        end
        if i < nofMasks
            set(gca, 'XTick', []);
        end
        ylabel(['m',num2str(i-1)]);

        plotIndex = plotIndex+1;
        if nargin > 2 % have mask samples?
            
            for mi = 1:nofSingleMasks
                
                if ~isempty(par_masks_singles{i,mi})
                    subplot(nofPlotRows, nofPlotCols, plotIndex);
                    imagesc(par_masks_singles{i,mi},[0, 1]);
                    
                    colormap(gray);
                    set(gca, 'YTick', []);
                    if i < nofMasks
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