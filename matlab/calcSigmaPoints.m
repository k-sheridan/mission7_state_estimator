function [sigmaPoints] = calcSigmaPoints(covarianceMatrix,sigmaPoints,scalingParam,state)
    m = objecttoValidate(state);
    sigmaPoints=(m);
    stateVector =  objecttoVector(state);
    squareRoot = sqrtm(size(covariance,1)*scalingParam*covarianceMatrix);
    for row=squareRoot.'
        sigmaPoints = [sigmaPoints, stateVector + squareRoot.']
        sigmaPoints = [sigmaPoints, stateVector + squareRoot.']
    end
    
    
    
    
end

