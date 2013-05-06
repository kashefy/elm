function [ output ] = load_weight_watch(par_str_weight_dir, par_str_filename_filter)

    K = dir(fullfile(par_str_weight_dir, ['*', par_str_filename_filter, '*']));
    nof_learners = length(K);
    weight_watch = cell(nof_learners, 1);
    
%     filenames = cell(1, length(K));
     for k = 1:length(K)
         filepath = fullfile(par_str_weight_dir, K(k).name);
         learner_weight_watch = dlmread(filepath);
         weight_watch{k} = learner_weight_watch';
     end
     output = weight_watch;
end