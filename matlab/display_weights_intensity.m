% Display weights
function [] = display_weights_intensity(par_weights, par_figure_id, par_masks)

[nof_learners, nof_features, nof_nodes] = size(par_weights);
nof_nodes_per_dim = floor(sqrt(nof_nodes));

[~, max_i] = max(par_weights,[], 2);
% drop singleton feature dim, since max is over feature dim
max_i = squeeze(max_i);

feat_step = 2/nof_features;
arr_feat_values = 0:feat_step:nof_features-feat_step;
arr_intensities = cell(1, nof_features);
for i = 1:nof_features

    arr_intensities{i} = i;
end

% subplot per learner, intensity = winning orientation
figure(par_figure_id);
nof_weighing_mech = 1;
nof_plot_rows = nof_weighing_mech;
nof_plot_cols = nof_learners; 
    
for i = 1:nof_learners
    
    if nargin < 3 % have masks?
        mask = ones(nof_nodes_per_dim);
    else
        mask = par_masks{i};
    end
    
    canvas = cell(nof_nodes_per_dim);
    k = 1;
    for r = 1:nof_nodes_per_dim
        for c = 1:nof_nodes_per_dim

            feat_index = max_i(i, k);
            canvas{r, c} = arr_feat_values(feat_index) * arr_intensities{feat_index} * mask(r, c);
            k = k+1;    
        end
    end
    to_disp = cell2mat(canvas);
    subplot(nof_plot_rows, nof_plot_cols, i)
    imagesc(to_disp, [arr_feat_values(1), arr_feat_values(nof_features)]);
    axis image
    set(gca, 'XTick', nof_nodes_per_dim, 'YTick', nof_nodes_per_dim);
    if i == nof_learners/2
        title('winning feature per pixel');
    end
end
colormap(jet(nof_features));
h = colorbar('location', 'EastOutside');
ylabel(h, 'intensity');
suptitle('weights per learner');
end
