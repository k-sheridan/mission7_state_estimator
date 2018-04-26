%initialize constants
speed = 0.33;
alpha = 0.01;
beta =2;
k = 3;
dt = 0.1;
d_total=40;
video = VideoWriter('movie.avi');
open(video);
s=rng;
data = [];


differenceMatrix = [];

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
newState = x;
reality = x;

%initialize H
H = [1,zeros(1,51);
     0,1,zeros(1,50);
     ];

measurement_uncertainty = 0.25

 
for k=4:2:40
   if rem(k,4) == 0
         continue;
   else
     H = [H;[zeros(1,k-2),1,zeros(1,53-k)]];
     H = [H;[zeros(1,k-1),1,zeros(1,52-k)]];
   end
end
for k=1:12
  if rem(k,3) == 0
     continue;
  else
     H = [H;[zeros(1,k+39),1,zeros(1,12-k)]];
  end   
end

%initialize process error Q
Q=diag(repelem(0.00001,52));

%initailize covariance matrix
thetaUncertainty = (pi/6)^2;
timerUncetainty = 9;
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
sizeCovariance = size(initCovarianceMatrix);
sizeX=size(objecttoVector(x));
newCovarianceMatrix = initCovarianceMatrix + Q;

%set lambda
lambda = alpha^2 * (size(initCovarianceMatrix,1) + k) - size(initCovarianceMatrix,1);

%initialize sigma points
sigmaPoints = calcSigmaPoints(initCovarianceMatrix,lambda,x);
transformedSigmaPoints = sigmaPoints;

%generate weights
weights = generateWeights(lambda,alpha,beta,size(newCovarianceMatrix,1));

for l=dt:dt:d_total
    %calculate sigma points
   transformedSigmaPoints = calcSigmaPoints(newCovarianceMatrix,lambda,newState);
   

    %run process
    for j=1:length(transformedSigmaPoints)
       transformedSigmaPoints(j) = process(transformedSigmaPoints(j),dt,dt);
    end
    
    %process state
    newState = process(newState,dt,dt);
    reality = process(reality,dt,dt);
    

    %calculate new covariance
    newCovarianceMatrix = unscentedTransform(weights,transformedSigmaPoints) + Q;
    %newCovarianceMatrix = (newCovarianceMatrix + newCovarianceMatrix.')/2;
    %num2str(newCovarianceMatrix)
    %symmetric = issymmetric(newCovarianceMatrix)
    
    %kalman update
    measurement = makeMeasurement(reality,0.1);
    [newState,newCovarianceMatrix] = kalmanUpdate(newState,newCovarianceMatrix,H,measurement,measurement_uncertainty);

    data = [data, objecttoVector(reality)-objecttoVector(newState)];

    %draw
    drawState(newState,newCovarianceMatrix,transformedSigmaPoints);
    frame = getframe(gcf);
    writeVideo(video,frame);
    
end

close(video);
plotxy_target(data,d_total,dt,newState);
plotxy_obstacle(data,d_total,dt,newState);
plotTheta_obstacle(data,d_total,dt,newState);
plotTheta_targets(data,d_total,dt,newState);
plotTimer(data,d_total,dt,newState);

