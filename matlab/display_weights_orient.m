% Display weights
function [] = display_weights_orient(par_weights, par_figure_id, par_masks)

[nof_learners, nof_features, nof_nodes] = size(par_weights);
nof_nodes_per_dim = floor(sqrt(nof_nodes));

[~, max_i] = max(par_weights,[], 2);
% drop singleton feature dim, since max is over feature dim
max_i = squeeze(max_i);

arr_orients = cell(1, nof_features);
angle_step = 180/nof_features;
orient_dim = 16;
arr_angle_deg = 0:angle_step:180-angle_step;
for i = 1:nof_features

    arr_orients{i} = gen_bar_stimulus(orient_dim, arr_angle_deg(i), 5);
end

% subplot per learner, intensity = winning orientation
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
            canvas{r, c} = arr_angle_deg(feat_index) * arr_orients{feat_index} * mask(r, c);
            k = k+1;    
        end
    end
    to_disp = cell2mat(canvas);
    plot_slot_start = (i-1)*nof_plot_slots_per_col + 1;
    plot_slot_end = plot_slot_start + nof_plot_slots_per_col-1;
    subplot(nof_plot_rows, nof_plot_slots_per_col*nof_plot_cols, [plot_slot_start, plot_slot_end])
    h = imagesc(to_disp, [arr_angle_deg(1), arr_angle_deg(nof_features)]);
    set(h, 'alphadata', to_disp > 0);
    axis image
    set(gca, 'XTick', [], 'YTick', []);
    if i == nof_learners/2
        title('winning feature per pixel');
    end
end
colormap(jet(nof_features));
h = colorbar('location', 'EastOutside');
set(gca, 'YTick', []);
ylabel(h, 'orientation[deg]');
suptitle('weights per learner');
end
