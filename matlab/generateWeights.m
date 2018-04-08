function [weights] = generateWeights(lambda,alpha,beta,n)
    initWeightMean = lambda/(n+lambda);
    initWeightCovariance = initWeightMean + (1 - alpha^2 + beta);
    weightI= 1/2*(n + lambda);  
    weights=[initWeightMean,initWeightCovariance,weightI];
end

