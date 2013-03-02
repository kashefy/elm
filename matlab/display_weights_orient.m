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
figure(par_figure_id)
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
            canvas{r, c} = feat_index * arr_orients{feat_index} * mask(r, c);
            k = k+1;    
        end
    end
    to_disp = cell2mat(canvas);
    subplot(nof_plot_rows, nof_plot_cols, i)
    h = imagesc(to_disp, [0, nof_features]);
    set(h, 'alphadata', to_disp>0);
    axis image
    set(gca, 'XTick', [], 'YTick', []);
    if(i == nof_learners/2)
        title('weights, colour = orientation')
    end
end
end
