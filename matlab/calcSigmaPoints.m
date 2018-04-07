function [sigmaPoints] = calcSigmaPoints(covarianceMatrix,scalingParam,state)
    sigmaPoints=(state);
    stateVector =  objecttoVector(state);
    squareRoot = sqrtm(size(covarianceMatrix,1)*scalingParam*covarianceMatrix);
    for row=squareRoot
        sigmaPoints = [sigmaPoints, vectortoObject(real(stateVector + row.'),state),vectortoObject(real(stateVector - row).',state)];
    end
end


