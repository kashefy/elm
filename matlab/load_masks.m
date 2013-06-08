function [ output, output_singles ] = load_masks(par_strFilenameMasks, par_strSingleMasksDir)

    nodes = dlmread(par_strFilenameMasks);
    [ nofMasks nofNodesPerMask ] = size(nodes);
    output = cell(nofMasks, 1);
    
    if nargin > 1
        
        K = dir(par_strSingleMasksDir);
        % assuming first two elements are '.' and '..', removing them
        K(1:2) = [];
        nofSingleMasks = 25;
        arrSingleMasks = cell(nofMasks, nofSingleMasks);

        filenames = cell(1, length(K));
        for k = 1:length(K) 
            filenames{k} =  K(k).name;
            filenameIndex = filenames{k};
            dot_indicies  = find(filenameIndex=='.');
            filenameIndex = filenameIndex(1:dot_indicies(1)-1);
            filenameIndex = str2double(filenameIndex)+1;
            
            if K(k).bytes > 0
                str_filepath = fullfile(par_strSingleMasksDir, filenames{k});
                [~, ~, ext] = fileparts(str_filepath);
                if strcmp(ext, '.csv')
                    values = dlmread(str_filepath, ',');
                else
                    fid = fopen(par_str_filepath);
                    dim = fread(fid, 1, 'int32', 0, 'b');
                    nof_cols = dim(1);
                    values = fread(fid, [nof_cols,inf], 'double', 0, 'b')';
                    fclose(fid);
                end
                singleMaskVals = values;
                
                [nofSingleMasksInFile, nofNodesPerSingleMask] = size(singleMaskVals);

                nofRows = floor(sqrt(nofNodesPerSingleMask));
                nofCols = floor(sqrt(nofNodesPerSingleMask));
                
                singleMaskIndicies = randperm(nofSingleMasksInFile);
                singleMaskVals = singleMaskVals(singleMaskIndicies, :);
                for mi = 1:min(nofSingleMasks, nofSingleMasksInFile)
                    arrSingleMasks{filenameIndex, mi} = reshape(singleMaskVals(mi, :), nofRows, nofCols)';
                end
            end
        end
        output_singles = arrSingleMasks;
    end
    
    parfor i=1:nofMasks

        maskNodes = nodes(i, :);

        nofRows = floor(sqrt(nofNodesPerMask));
        nofCols = floor(sqrt(nofNodesPerMask));

        featureWeights = reshape(maskNodes, nofRows, nofCols)';
        output{i} = featureWeights;
    end
end