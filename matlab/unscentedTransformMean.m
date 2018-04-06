function [meanVector] = unscentedTransformMean(weights,sigmaPoints)
    initWeightsMean = weights(1);
    meanVector = initWeightsMean*objecttoVector(sigmaPoints(1));
    for j=2:length(sigmaPoints)
        meanVector= meanVector + weights(3) * objecttoVector(sigmaPoints(j));
        meanVecSize = size(meanVector)
        pointSize =size(objecttoVector(sigmaPoints(j)))
    end
end

