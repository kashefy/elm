% Display weights
function [ results ] = read_weights(par_strFilename, par_slicing)

weights = dlmread(par_strFilename);
[nof_learners ~] = size(weights);
nof_chunks = length(par_slicing);
arr_weights_chunk = cell(nof_chunks, 1);

for i_c = 1:nof_chunks
    
    chunk = par_slicing{i_c};
    if i_c == 1
        i_start = 1;
    else
        i_start = i_start_prev+1;
    end
    chunk_len = chunk(1);
    i_start_prev = chunk_len;
    weights_chunk = weights(:, i_start:i_start+chunk_len-1);
    nof_features = chunk(2);
    arr_weights_chunk{i_c} = reshape(weights_chunk, nof_learners, nof_features, chunk_len/nof_features);
end
results = arr_weights_chunk;
end
