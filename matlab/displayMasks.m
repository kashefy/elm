function [ output, output_singles ] = displayMasks(par_strFilenameMasks, par_strSingleMasksDir)

    nodes = dlmread(par_strFilenameMasks);
    [ nofMasks nofNodesPerMask ] = size(nodes);
    output = cell(nofMasks, 1);
    nofPlotRows = nofMasks;
    nofPlotCols = 1;
    %clims = [min(nodes(:)), max(nodes(:))];
    clims = [0, 1];
    
    if nargin > 1
        
        nofSingleMasks = 8;
        nofPlotCols = nofPlotCols + nofSingleMasks;

        K = dir(par_strSingleMasksDir);
        % assuming first two elements are '.' and '..', removing them
        K(1:2)=[];
        
        arrSingleMasks = cell(nofMasks,nofSingleMasks);

        filenames = cell(1, length(K));
        for k = 1:length(K) 
            filenames{k} =  K(k).name;
            filenameIndex = filenames{k};
            dot_indicies  = find(filenameIndex=='.');
            filenameIndex = filenameIndex(1:dot_indicies(1)-1);
            filenameIndex = str2num(filenameIndex)+1;
            
            if K(k).bytes > 0
                singleMaskVals = dlmread([par_strSingleMasksDir, filenames{k}]);

                [nofSingleMasksInFile, nofNodesPerSingleMask] = size(singleMaskVals);

                nofRows = floor(sqrt(nofNodesPerSingleMask));
                nofCols = floor(sqrt(nofNodesPerSingleMask));
                
                singleMaskIndicies = [];
                while length(singleMaskIndicies) < min(nofSingleMasks, nofSingleMasksInFile)
                    
                    singleMaskIndicies = randi(nofSingleMasksInFile,1,nofSingleMasks);
                    singleMaskIndicies = unique(singleMaskIndicies);
                end
                singleMaskVals = singleMaskVals(singleMaskIndicies,:);
                for mi = 1:min(nofSingleMasks, nofSingleMasksInFile)
                    arrSingleMasks{filenameIndex,mi} = reshape(singleMaskVals(mi,:),nofRows,nofCols)';
                end
            end
        end
        output_singles = arrSingleMasks;
    end
    
    plotIndex = 1;
    for i=1:nofMasks

        maskNodes = nodes(i,:);

        nofRows = floor(sqrt(nofNodesPerMask));
        nofCols = floor(sqrt(nofNodesPerMask));

        featureWeights = reshape(maskNodes,nofRows,nofCols)';

        subplot(nofPlotRows, nofPlotCols, plotIndex);
        imagesc(featureWeights, clims);
        colormap(gray);
        axis image
        if i == nofMasks
            set(gca,'XTick',[1 nofCols]);
        end
        set(gca,'YTick',[1 nofRows]);
        if(i==1)
            title('aggregate masks');
        end
        if nargin > 1
            ylabel(['z',num2str(i-1)]);
        else
            ylabel(['c',num2str(i-1)]);
        end
        
        output{i} = featureWeights;

        plotIndex = plotIndex+1;
        
        if nargin > 1
   
            for mi = 1:nofSingleMasks
                
                if(~isempty(arrSingleMasks{i,mi}))
                    subplot(nofPlotRows, nofPlotCols, plotIndex);
                    imagesc(arrSingleMasks{i,mi},[0, 1]);
                    
                    colormap(gray);
                    set(gca,'XTick',[]);
                    set(gca,'YTick',[]);
                    axis image
                    if(i==1 && mi == ceil(nofSingleMasks/2))
                        title('mask samples');
                    end
                end

                plotIndex = plotIndex+1;
            end
        end
    end
    
    
end