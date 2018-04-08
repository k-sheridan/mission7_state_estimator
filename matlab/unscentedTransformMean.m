function [meanVector] = unscentedTransformMean(weights,sigmaPoints)
    initWeightsMean = weights(1);
    meanVector = initWeightsMean*objecttoVector(sigmaPoints(1));
    for j=2:length(sigmaPoints)
        pointSize =size(objecttoVector(sigmaPoints(j)));
        meanVector= meanVector + weights(3)*objecttoVector(sigmaPoints(j)); 
    end
    meanVector = real(meanVector);
end

