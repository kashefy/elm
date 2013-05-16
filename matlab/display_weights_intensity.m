% Display weights
function [] = display_weights_intensity(par_weights, par_figure_id, par_masks)

[nof_learners, nof_features, nof_nodes] = size(par_weights);
nof_nodes_per_dim = floor(sqrt(nof_nodes));

[~, max_i] = max(par_weights, [], 2);
% drop singleton feature dim, since max is over feature dim
max_i = squeeze(max_i);

feat_step = 2/nof_features;
arr_feat_values = 0:feat_step:nof_features-feat_step;
arr_intensities = cell(1, nof_features);
for i = 1:nof_features

    arr_intensities{i} = i;
end

% subplot per learner, intensity = winning feature
figure(par_figure_id);
nof_weighing_mech = 1;
nof_plot_rows = nof_weighing_mech;
nof_plot_cols = nof_learners;
nof_plot_slots_per_col = 4;
    
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
    
    plot_slot_start = (i-1)*nof_plot_slots_per_col + 1;
    plot_slot_end = plot_slot_start + nof_plot_slots_per_col - 1;
    sub_plot = subplot(nof_plot_rows, nof_plot_slots_per_col*nof_plot_cols+4, [plot_slot_start, plot_slot_end]);
    
    imagesc(to_disp, [arr_feat_values(1), arr_feat_values(nof_features)]);
    if i == 1
        set(gca, 'YTick', nof_nodes_per_dim);
    else
        set(gca, 'YTick', []);
    end
    set(gca, 'XTick', nof_nodes_per_dim);
    if i == nof_learners/2
        title('winning feature per pixel');
    end
    axis image
end
% add colorbar while preserving subplot size
pos_gca = get(sub_plot, 'position');
colormap(jet(nof_features));
handle_colorbar = colorbar('location', 'EastOutside');
ylabel(handle_colorbar, 'intensity');
set(sub_plot, 'position', pos_gca);
set(handle_colorbar, 'location', 'EastOutside');
suptitle('weights per learner');

set(gcf, 'PaperPositionMode', 'auto');
set(gcf, 'units', 'normalized', 'outerposition', [0 0 1 1]);
saveas(gcf, 'weights_pl_intensity.png');
% produce weight value figures

% subplot per learner, intensity = winning feature intenisty
figure(par_figure_id+1);
nof_plot_rows = nof_features;
min_weight = min(par_weights(:));
max_weight = max(par_weights(:));
    
for i = 1:nof_learners
    
    if nargin < 3 % have masks?
        mask = ones(nof_nodes_per_dim);
    else
        mask = par_masks{i};
    end
    
    for fi = 1:nof_features
        weights = reshape(squeeze(par_weights(i, fi, :)), nof_nodes_per_dim, nof_nodes_per_dim)';
        to_disp = weights .* mask;
    
        plot_feature_offset = nof_plot_slots_per_col*nof_plot_cols*(fi-1);
        plot_slot_start = (i-1)*nof_plot_slots_per_col + 1;
        plot_slot_end = plot_slot_start + nof_plot_slots_per_col - 1;
        sub_plot = subplot(nof_plot_rows, nof_plot_slots_per_col*nof_plot_cols, [plot_slot_start, plot_slot_end]+plot_feature_offset);
        imagesc(to_disp, [min_weight, max_weight]);
        if i == 1
            set(gca, 'YTick', nof_nodes_per_dim);
        else
            set(gca, 'YTick', []);
        end
        set(gca, 'XTick', nof_nodes_per_dim);
        if i == nof_learners/2 && fi == 1
            title('Off');
        elseif i == nof_learners/2 && fi == 2
            title('On');
        end
        axis image
    end
end
% add colorbar while preserving subplot size
pos_gca = get(sub_plot, 'position');
colormap('gray')
handle_colorbar = colorbar('location', 'EastOutside');
ylabel(handle_colorbar, 'weight');
set(sub_plot, 'position', pos_gca);
set(handle_colorbar, 'location', 'EastOutside');
pos_colorbar = get(handle_colorbar, 'position');
set(handle_colorbar, 'position', [pos_colorbar(1:3), pos_gca(4)]);
suptitle('weights per learner per feature');

set(gcf, 'PaperPositionMode', 'auto');
set(gcf, 'units', 'normalized', 'outerposition', [0 0 1 1]);
saveas(gcf, 'weights_pl_pf_intensity.png');

end
