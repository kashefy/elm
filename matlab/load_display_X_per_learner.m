function [] = load_display_X_per_learner(par_str_filepath, par_figure_offset)

     [~, ~, ext] = fileparts(par_str_filepath);
     if strcmp(ext, '.csv')
         values = dlmread(par_str_filepath, ',');
     else
        fid = fopen(par_str_filepath);
        dim = fread(fid, 1, 'int32', 0, 'b');
        nof_cols = dim(1);
        values = fread(fid, [nof_cols,inf], 'double', 0, 'b')';
        fclose(fid);
     end
    
    nof_plot_rows = 2;
    nof_plot_cols = 1;
    figure(par_figure_offset)
    subplot(nof_plot_rows, nof_plot_cols, 1);
    plot(values);
    subplot(nof_plot_rows, nof_plot_cols, 2);
    imagesc(values');
    

    set(gcf, 'PaperPositionMode', 'auto');
    set(gcf, 'units', 'normalized', 'outerposition', [0 0 1 1]);
    %saveas(gcf, ['weight_watch_', par_str_filename_filter, '.png']);
end