function [ firingProbs, condEntropy, preferredCause] = load_prediction_stats(par_filepath)

predictionStats = dlmread(par_filepath);
[ nofLearners nofCols ] = size(predictionStats);
firingProbs = predictionStats(:, 1:nofCols-1);
condEntropy = predictionStats(:, nofCols);

[byHowMuch, preferredCause] = max(firingProbs');
% sum(firingProbs, 1)
% sum(firingProbs, 2)
preferredCause(byHowMuch==0) = 0;

end
