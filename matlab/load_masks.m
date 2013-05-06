function [ output, output_singles ] = load_masks(par_strFilenameMasks, par_strSingleMasksDir)

    nodes = dlmread(par_strFilenameMasks);
    [ nofMasks nofNodesPerMask ] = size(nodes);
    output = cell(nofMasks, 1);
    
    if nargin > 1
        
        K = dir(par_strSingleMasksDir);
        % assuming first two elements are '.' and '..', removing them
        K(1:2)=[];
        nofSingleMasks = 25;
        arrSingleMasks = cell(nofMasks, nofSingleMasks);

        filenames = cell(1, length(K));
        for k = 1:length(K) 
            filenames{k} =  K(k).name;
            filenameIndex = filenames{k};
            dot_indicies  = find(filenameIndex=='.');
            filenameIndex = filenameIndex(1:dot_indicies(1)-1);
            filenameIndex = str2num(filenameIndex)+1;
            
            if K(k).bytes > 0
                singleMaskVals = dlmread(fullfile(par_strSingleMasksDir, filenames{k}));

                [nofSingleMasksInFile, nofNodesPerSingleMask] = size(singleMaskVals);

                nofRows = floor(sqrt(nofNodesPerSingleMask));
                nofCols = floor(sqrt(nofNodesPerSingleMask));
                
                singleMaskIndicies = [];
                while length(singleMaskIndicies) < min(nofSingleMasks, nofSingleMasksInFile)
                    
                    singleMaskIndicies = randi(nofSingleMasksInFile, 1, nofSingleMasks);
                    singleMaskIndicies = unique(singleMaskIndicies);
                end
                singleMaskIndicies = sort(singleMaskIndicies);
                singleMaskVals = singleMaskVals(singleMaskIndicies, :);
                for mi = 1:min(nofSingleMasks, nofSingleMasksInFile)
                    arrSingleMasks{filenameIndex,mi} = reshape(singleMaskVals(mi,:), nofRows, nofCols)';
                end
            end
        end
        output_singles = arrSingleMasks;
    end
    
    for i=1:nofMasks

        maskNodes = nodes(i, :);

        nofRows = floor(sqrt(nofNodesPerMask));
        nofCols = floor(sqrt(nofNodesPerMask));

        featureWeights = reshape(maskNodes, nofRows, nofCols)';
        output{i} = featureWeights;
    end
end