%check H
b = 21;
m = H * objecttoVector(x);
for target=x.obstacle_robots
    if m(b)==target.state(1)
        b=b+1;
    else
        b
        break
    end
    if m(b)==target.state(2)
        b=b+1;
    else
        b
        break
    end
end
for target=x.target_robots
    if m(b)==target.state(1)
        b=b+1;
    else
        b
        break
    end
    if m(b)==target.state(2)
        b=b+1;
    else
        b
        break
    end
end



%check weights
   y = weights(1);
   for l=2:length(transformedSigmaPoints)
       %y = y+weights(3);
   end
   y

%check mean
checkMean = unscentedTransformMean(weights,sigmaPoints);
boolCheckMean = isequal(checkMean,objecttoVector(x))
meanCompare = [checkMean, objecttoVector(x)]

%test covariance
checkCovarianceMatrix = unscentedTransform(weights,sigmaPoints);
checkCovarianceMatrix = (checkCovarianceMatrix + checkCovarianceMatrix.')/2;
checkCovariance = num2str(checkCovarianceMatrix)
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