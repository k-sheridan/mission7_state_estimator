function [covariance] = uncscentedTransform(weights,sigmaPoints)
    initCovarianceWeight = weights(2);
    meanVector = unscentedTransformMean(weights,sigmaPoints);
    covariance = initCovarianceWeight * (objecttoVector(sigmaPoints(0))-meanVector) * (objecttoVector(sigmaPoints(0))-meanVector).'
    for j=2:sigmaPoints
        covariance = covariance+weights(3)*(objecttoVector(sigmaPoints(j))-meanVector) * (objecttoVector(sigmaPoints(j))-meanVector).'
    end
end


