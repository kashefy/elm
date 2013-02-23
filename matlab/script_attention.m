%% convert attention window data from csv to png

strMainPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\output\\MNIST';
strAttentionDir = 'att';
str_attention_window_prefix = 'window_';
str_attention_window_ext = '.csv';
str_attention_window_ext_img = '.png';

for i_window = 0:100000
    
    str_attention_window_filepath_csv = fullfile(strMainPath, strAttentionDir, [str_attention_window_prefix, num2str(i_window), str_attention_window_ext]);
    
    status = exist(str_attention_window_filepath_csv, 'file');
    
    if status == 2 % name is the full pathname to any file
        
        attention_window = dlmread(str_attention_window_filepath_csv);
        attention_window_filepath_png = fullfile(strMainPath, strAttentionDir, [str_attention_window_prefix, num2str(i_window), str_attention_window_ext_img]);
        imwrite(attention_window, attention_window_filepath_png);
        
    end
    
end

%% display some attention windows

strMainPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\output\\MNIST';
strAttentionDir = 'att';
str_attention_window_prefix = 'window_';
str_attention_window_ext = '.csv';

figure(20002)
nof_subplot_rows = 8;
nof_subplot_cols = 8;

nof_subplots = nof_subplot_rows * nof_subplot_cols;

for i_window = 0:nof_subplots
    
    i_subplot = i_window+1;
    
    str_attention_window_filepath_csv = fullfile(strMainPath, strAttentionDir, [str_attention_window_prefix, num2str(i_window), str_attention_window_ext]);
    
    status = exist(str_attention_window_filepath_csv, 'file');
    
    if status == 2 % name is the full pathname to any file
        
        attention_window = dlmread(str_attention_window_filepath_csv);
        subplot(nof_subplot_rows, nof_subplot_cols, i_subplot)
        imagesc(attention_window)
        axis image
        
    end
    
end

%% display attention window location centre histogram

strMainPath = 'C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\output\\MNIST';
strAttentionDir = 'att';
str_loc_attended_count_centre_filename = 'loc_attended_count_centre.csv';
loc_attended_count_centre = dlmread(fullfile(strMainPath, strAttentionDir, str_loc_attended_count_centre_filename));
figure(20003)
imagesc(loc_attended_count_centre)
colorbar