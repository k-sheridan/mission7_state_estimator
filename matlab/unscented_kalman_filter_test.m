speed = 0.33;
alpha = 0.3;
beta =-0.91;
k = 0;



for n = 0:9
    targets(n+1) = TargetRobot([0.5*cos(36 * n * pi/180); 0.5*sin(36 * n * pi/180); 36 * n * pi/180; 0],0,n+1);
end

for n = 0:3
    obstacles(n+1) = ObstacleRobot([5*cos(90 * n * pi/180); 5*sin(90 * n * pi/180); 90 * n * pi/180 - pi/2],0,n+1); 
end

x = State;

x.target_robots = targets;
x.obstacle_robots = obstacles;
 
dt = 0.1;

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

lambda = alpha^2 * (size(initCovarianceMatrix,1) + k) - size(initCovarianceMatrix,1)


   sigmaPoints = calcSigmaPoints(initCovarianceMatrix,lambda,x);
   for j=1:length(sigmaPoints)
       sigmaPoints(j) = process(sigmaPoints(j),10,dt);
   end
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



