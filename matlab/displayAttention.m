function [ output ] = displayAttention(par_strPath, par_strSuffix, par_figureOffset)

strPath = par_strPath;
strSuffix = par_strSuffix;
figureOffset = par_figureOffset;

    figure(figureOffset+1);
    wAvgFiringSum = dlmread([strPath, 'wAvgFiringSum' , strSuffix, '.csv']);
    imagesc(wAvgFiringSum)
    title('wAvgFiringSum');

    % excourse

    [ nofAttShiftsY, nofAttShiftsX ] = size(wAvgFiringSum);
    wFiringCounts = dlmread([strPath, 'wFiringCounts', strSuffix, '.csv']);
    maxFiringCount = max(max(wFiringCounts));
    minFiringCount = min(min(wFiringCounts));
    [ nofAttshifts, nofLearners ] =  size(wFiringCounts);
    wFiringProbs = dlmread([strPath, 'wFiringProbs', strSuffix, '.csv']);
    indicies = [1, randi(nofAttshifts-1,1,7)+1];

    figure(figureOffset+2);
    scene = dlmread([strPath, '\att\scene.csv']);
    imagesc(scene);
    title('scene');
    figure(figureOffset+3);
    wAvgCondEntropy = dlmread([strPath, 'wAvgCondEntropy', strSuffix, '.csv']);
    imagesc(wAvgCondEntropy);
    title(['wAvgCondEntropy ',num2str(nofLearners), ' learners']);
    figure(figureOffset+4);
    winnerPerWindow = dlmread([strPath, 'winnerPerWindow', strSuffix, '.csv']);
    subplot(1,2,1);
    imagesc(winnerPerWindow,[0, nofLearners-1]);
    title('winnerPerWindow');
    %figure(figureOffset+5);
    winningPerc = dlmread([strPath, 'winningPerc', strSuffix, '.csv']);
    subplot(1,2,2);
    imagesc(winningPerc,[0,1]);
    title('winningPerc');

    figure(figureOffset+6);
    nofPlotRows = length(indicies);
    nofPlotCols = 2;
    for i=1:length(indicies)
        subplot(nofPlotRows,nofPlotCols,(i-1)*nofPlotCols+1);
        %plot(0:nofLearners-1,wFiringCounts(indicies(i),:));
        imagesc(wFiringCounts(indicies(i),:),[minFiringCount, maxFiringCount]);
        set(gca,'YTick',[]);
        if i==1
           title('wFiringCounts'); 
        end
        %xlabel(['z @ ',num2str(indicies(i)),'']);
        %set(gca,'XTick',0:1:nofLearners);
        if i==length(indicies)
            xlabel('z');
        end
        ylabel(['@ ',num2str(indicies(i)),'']);
        subplot(nofPlotRows,nofPlotCols,(i-1)*nofPlotCols+2);
        %plot(0:nofLearners-1,wFiringProbs(indicies(i),:));
        %ylim([0 1]);
        imagesc(wFiringProbs(indicies(i),:),[0,1]);
        set(gca,'YTick',[]);
        if i==1
           title('wFiringProbs'); 
        end
        %xlabel(['z @ ',num2str(indicies(i)),'']);
        if i==length(indicies)
            xlabel('z');
        end

    end

    wFiringCountsFalse = wFiringCounts(2:nofAttshifts,:);
    wFiringProbsFalse = wFiringProbs(2:nofAttshifts,:);
    avgFiringCountsFalse = mean(wFiringCountsFalse);
    avgFiringProbsFalse = mean(wFiringProbsFalse);

    figure(figureOffset+7);
    nofPlotRows = 1;
    nofPlotCols = 2;
    subplot(nofPlotRows,nofPlotCols,1);
    imagesc(avgFiringCountsFalse,[minFiringCount, maxFiringCount]);
    set(gca,'YTick',[]);
    title('avgFiringCountsFalse'); 
    if i==length(indicies)
        xlabel('z');
    end
    subplot(nofPlotRows,nofPlotCols,2);
    imagesc(avgFiringProbsFalse,[0,1]);
    set(gca,'YTick',[]);
    title('avgFiringProbsFalse'); 
    if i==length(indicies)
        xlabel('z');
    end

    %
    figure(figureOffset+8);
    subplot(2,2,1);
    imagesc(winnerPerWindow,[0, nofLearners-1]);
    title('winnerPerWindow');
    subplot(2,2,2);
    imagesc(winningPerc,[0,1]);
    title('winningPerc');

    [~, window0winZ] = max(wFiringCounts(1,:));
    window0winZFiringCounts = reshape(wFiringCounts(:,window0winZ),nofAttShiftsX,nofAttShiftsY)';
    subplot(2,2,3);
    imagesc(window0winZFiringCounts,[minFiringCount, maxFiringCount]);
    title(['window0winZFiringCount z', num2str(window0winZ-1)]);
    window0winZFiringProb = reshape(wFiringProbs(:,window0winZ),nofAttShiftsX,nofAttShiftsY)';
    subplot(2,2,4);
    imagesc(window0winZFiringProb,[0,1]);
    title(['window0winZFiringProb z', num2str(window0winZ-1)]);

end