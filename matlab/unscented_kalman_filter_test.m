speed = 0.33;
alpha = 0.99;
beta =2;
k = 165;



for n = 0:9
    targets(n+1) = TargetRobot([0.5*cos(36 * n * pi/180); 0.5*sin(36 * n * pi/180); 36 * n * pi/180; 0],0,n+1);
end

for n = 0:3
    obstacles(n+1) = ObstacleRobot([5*cos(90 * n * pi/180); 5*sin(90 * n * pi/180); 90 * n * pi/180 - pi/2; 0],0,n+1); 
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

lambda = alpha^2 * (size(initCovarianceMatrix,1) + k) - size(initCovarianceMatrix,1);

for t = (0:dt:120)
   sigmaPoints = calcSigmaPoints(initCovarianceMatrix,lambda,x);
   weights = generateWeights(lambda,alpha,beta,size(initCovarianceMatrix,1));
   x = unscentedTransformMean(weights,sigmaPoints);
   initCovarianceMatrix = unscentedTransform(weights,sigmaPoints);
   drawState(x,initCovarianceMatrix)
end



