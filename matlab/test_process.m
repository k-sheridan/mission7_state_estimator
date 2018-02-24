speed = 0.33;

for n = 0:9
    targets(n+1) = TargetRobot([cos(36 * n/180 * pi); sin(36 * n/180 * pi); 36 * n/180 * pi; 0],0);
end

for n = 0:3
    obstacles(n+1) = ObstacleRobot([5*cos(90 * n/180 * pi); 5*sin(90 * n/180 * pi); 90 * n/180 * pi - pi/2; 0],0); 
end

x = State;

x.target_robots = targets;
x.obstacle_robots = obstacles;

drawState(x)