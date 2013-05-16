function [] = load_display_weight_watch(par_str_weight_dir, par_str_filename_filter, par_figure_offset)

    K = dir(fullfile(par_str_weight_dir, ['*', par_str_filename_filter, '*']));
    nof_learners = length(K);
    
    nof_plot_rows = nof_learners;
    nof_plot_cols = 1;
    nof_plot_slots_per_row = 2;
    figure(par_figure_offset)
    
     for k = 1:length(K)
         filepath = fullfile(par_str_weight_dir, K(k).name);
         filename = K(k).name;
         dot_indicies  = find(filename=='.');
         under_indicies = find(filename=='_');
         str_index = filename(under_indicies(end)+1:dot_indicies(end)-1);
         i = str2double(str_index)+1;
         
         plot_slot_start = (i-1)*nof_plot_slots_per_row + 1;
         plot_slot_end = plot_slot_start + nof_plot_slots_per_row-1;
         
         subplot(nof_plot_slots_per_row*nof_plot_rows, nof_plot_cols, [plot_slot_start, plot_slot_end])
         [~, ~, ext] = fileparts(filepath);
         if strcmp(ext, '.csv')
             learner_weight_watch = dlmread(filepath);
             imagesc(learner_weight_watch');
         else
            fid = fopen(filepath);
            dim = fread(fid, 1, 'int32', 0, 'b');
            nof_cols = dim(1);
            learner_weight_watch = fread(fid, [nof_cols, inf], 'double', 0, 'b');
            imagesc(learner_weight_watch);%(:,1:1:end));
            fclose(fid);
         end
         
         
         ylabel(num2str(i));
         if i < nof_learners
             set(gca, 'XTick', [])
         end
     end
     
     set(gcf, 'PaperPositionMode', 'auto');
     set(gcf, 'units', 'normalized', 'outerposition', [0 0 1 1]);
     saveas(gcf, ['weight_watch_', par_str_filename_filter, '.png']);
end