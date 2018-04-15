alpha =0.1;
beta =2;
k = 0;

%init constants
covarianceMatrix = [1,0;0,1];
lambda = alpha^2 * (size(covarianceMatrix,1) + k) - size(covarianceMatrix,1);
state=[1;1];

%calculate sigmapoints
sigmaPoints=(state);
squareRoot = real(sqrtm((size(covarianceMatrix,1)+lambda)*covarianceMatrix));
squareRoot
for row=squareRoot.'
   sigmaPoints = [sigmaPoints (state + row) (state - row)];
end

%run process
processSigmaPoints = sigmaPoints
for l=1:size(sigmaPoints,2);
    F = [1, 0.1; 0, 1];
    processSigmaPoints(:,l)=F*sigmaPoints(:,l);
end
processSigmaPoints
sigmaPoints

%calculate and confirm wieghts
weights = generateWeights(lambda,alpha,beta,size(covarianceMatrix,1))
y = weights(1);
 for l=2:size(sigmaPoints,2)
   y= y+weights(3);
 end
y


%calculate mean
initWeightsMean = weights(1);
meanVector = initWeightsMean*processSigmaPoints(:,1);
for j=2:length(processSigmaPoints)
   meanVector= meanVector + weights(3)*processSigmaPoints(:,j); 
end
meanVector = real(meanVector);

%check weights sigma points and mean process
checkMeanVector = initWeightsMean*sigmaPoints(:,1);
for j=2:length(sigmaPoints)
   checkMeanVector= checkMeanVector + weights(3)*sigmaPoints(:,j); 
end
checkMeanVector = real(checkMeanVector)
state


%calculate covariance
 finalCovariance = weights(2)*(processSigmaPoints(:,1)-meanVector)*(processSigmaPoints(:,1)-meanVector).';
    for j=2:size(processSigmaPoints,2)
        finalCovariance = finalCovariance+weights(3)*(processSigmaPoints(:,j)-meanVector)*(processSigmaPoints(:,j)-meanVector).';
    end
    finalCovariance = real(finalCovariance)


%check covariance process
 checkCovariance = weights(2)*(sigmaPoints(:,1)-checkMeanVector)*(sigmaPoints(:,1)-checkMeanVector).';
    for j=2:size(processSigmaPoints,2)
        checkCovariance = checkCovariance+weights(3)*(sigmaPoints(:,j)-checkMeanVector)*(sigmaPoints(:,j)-checkMeanVector).';
    end
    checkCovariance = real(checkCovariance)

