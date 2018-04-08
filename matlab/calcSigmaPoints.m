function [sigmaPoints] = calcSigmaPoints(covarianceMatrix,scalingParam,state)
    sigmaPoints=(state);
    stateVector =  objecttoVector(state);
    squareRoot = real(sqrtm((size(covarianceMatrix,1)+scalingParam)*covarianceMatrix));
    for row=squareRoot.'
        sigmaPoints = [sigmaPoints, vectortoObject(stateVector + row.',state),vectortoObject(stateVector - row.',state)];
    end
end


