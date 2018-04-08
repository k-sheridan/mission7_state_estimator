alpha = 0.1;
beta =2;
k = 0;

initCovarianceMatrix = [1,0;0,1];
lambda = alpha^2 * (size(initCovarianceMatrix,1) + k) - size(initCovarianceMatrix,1);

sigmaPoints = calcSigmaPoints(initCovarianceMatrix,lambda,x);
   weights = generateWeights(lambda,alpha,beta,size(initCovarianceMatrix,1))
   y = weights(2);
   for l=2:length(sigmaPoints)
       y = y+weights(3);
   end
   newState = unscentedTransformMean(weights,sigmaPoints);
   newState = vectortoObject(newState,x);
   initCovarianceMatrix = unscentedTransform(weights,sigmaPoints);
   initCovarianceMatrix = (initCovarianceMatrix + initCovarianceMatrix.')/2;
   num2str(initCovarianceMatrix)
   symmetric = issymmetric(initCovarianceMatrix)
   drawState(newState,initCovarianceMatrix,sigmaPoints);
