%initialize constants
speed = 0.33;
alpha = 0.1;
beta =2;
k = 0;
dt = 0.1;

%build state
for n = 0:9
    targets(n+1) = TargetRobot([0.5*cos(36 * n * pi/180); 0.5*sin(36 * n * pi/180); 36 * n * pi/180; 0],0,n+1);
end

for n = 0:3
    obstacles(n+1) = ObstacleRobot([5*cos(90 * n * pi/180); 5*sin(90 * n * pi/180); 90 * n * pi/180 - pi/2],0,n+1); 
end

x = State;

x.target_robots = targets;
x.obstacle_robots = obstacles;

%initailize covariance matrix
thetaUncertainty = (pi/20)^2;
timerUncetainty = 100;
xUncertainty = 0.25;
yUncertainty = 0.25;
diagArray=[];
for j = 1:14
    if j<=10
        diagArray=[diagArray [xUncertainty,yUncertainty,thetaUncertainty,timerUncetainty]];
    else
        diagArray = [diagArray [xUncertainty,yUncertainty,thetaUncertainty]];
    end
end
initCovarianceMatrix = diag(diagArray);
sizeCovariance = size(initCovarianceMatrix)
sizeX=size(objecttoVector(x))

%set lambda
lambda = alpha^2 * (size(initCovarianceMatrix,1) + k) - size(initCovarianceMatrix,1)

%calculate sigma points
   sigmaPoints = calcSigmaPoints(initCovarianceMatrix,lambda,x);
   
%run process
   transformedSigmaPoints = sigmaPoints;
   for j=1:length(transformedSigmaPoints)
       transformedSigmaPoints(j) = process(transformedSigmaPoints(j),10,dt);
   end
   
%generate weights
   weights = generateWeights(lambda,alpha,beta,size(initCovarianceMatrix,1))
   y = weights(1);

%check weights
   for l=2:length(transformedSigmaPoints)
       y = y+weights(3);
   end
   y

%get new mean
newState = unscentedTransformMean(weights,transformedSigmaPoints);
newState = vectortoObject(newState,x);
   
%check mean
checkMean = unscentedTransformMean(weights,sigmaPoints);
boolCheckMean = isequal(checkMean,objecttoVector(x))
meanCompare = [checkMean, objecttoVector(x)]

 

%calculate new covariance
newCovarianceMatrix = unscentedTransform(weights,transformedSigmaPoints);
newCovarianceMatrix = (newCovarianceMatrix + newCovarianceMatrix.')/2;
num2str(newCovarianceMatrix)
symmetric = issymmetric(newCovarianceMatrix)

%test covariance
checkCovarianceMatrix = unscentedTransform(weights,sigmaPoints);
checkCovarianceMatrix = (checkCovarianceMatrix + checkCovarianceMatrix.')/2;
checkCovariance = num2str(newCovarianceMatrix)
symmetric = issymmetric(checkCovarianceMatrix)

%test objecttoVector 
for n = 0:9
    targets(n+1) = TargetRobot([10*n+1; 10*n+2; 10*n+3; 10*n+4],0,n+1);
end

for n = 0:3
    obstacles(n+1) = ObstacleRobot([100*n+1; 100*n+2; 100*n+3],0,n+1); 
end

test = State;

test.target_robots = targets;
test.obstacle_robots = obstacles;
testVector=[];
for n = 0:9
    testVector = [testVector; [10*n+1; 10*n+2; 10*n+3; 10*n+4]];
end

for n = 0:3
    testVector = [testVector; [100*n+1; 100*n+2; 100*n+3]];
end
testVector;
boolObjecttoVector = isequal(testVector,objecttoVector(test))
checkObjecttoVector = [testVector,objecttoVector(test)]

%test vectortoObject
boolvectortoObject = isequal(objecttoVector(vectortoObject(testVector,test)),objecttoVector(test))
checkVectortoObject=[objecttoVector(vectortoObject(testVector,test)),objecttoVector(test)]

%draw
 drawState(newState,newCovarianceMatrix,transformedSigmaPoints);




