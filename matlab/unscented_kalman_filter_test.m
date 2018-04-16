%initialize constants
speed = 0.33;
alpha = 0.01;
beta =2;
k = 3;
dt = 0.1;
d_total=40;
video = VideoWriter('movie.avi');
open(video);

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

%initailize covariance matrix
thetaUncertainty = (pi/6)^2;
timerUncetainty = 3^2;
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
newCovarianceMatrix = initCovarianceMatrix;

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
    
    

    %calculate new covariance
    newCovarianceMatrix = unscentedTransform(weights,transformedSigmaPoints);
    %newCovarianceMatrix = (newCovarianceMatrix + newCovarianceMatrix.')/2;
    num2str(newCovarianceMatrix)
    %symmetric = issymmetric(newCovarianceMatrix)



    %draw
    drawState(newState,newCovarianceMatrix,transformedSigmaPoints);
    frame = getframe(gcf);
    writeVideo(video,frame);
    
end

close(video);



