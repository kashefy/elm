function [ output ] = display_weight_watch(par_weight_watch, par_figure_offset)

    nof_learners = length(par_weight_watch);
    nof_plot_rows = nof_learners;
    nof_plot_cols = 1;
    nof_plot_slots_per_row = 4;
    figure(par_figure_offset)
     for i = 1:nof_learners
         %subplot(nof_plot_rows, 1, i)
         plot_slot_start = (i-1)*nof_plot_slots_per_row + 1;
         plot_slot_end = plot_slot_start + nof_plot_slots_per_row-1;
         subplot(nof_plot_slots_per_row*nof_plot_rows, nof_plot_cols, [plot_slot_start, plot_slot_end])
         imagesc(par_weight_watch{i})
         %axis tight
     end
end