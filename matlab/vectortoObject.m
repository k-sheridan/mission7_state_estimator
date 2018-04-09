function [state] = vectortoObject(vector,lastState)
   targets= [];
   obstacles = [];
   state = State;
   for j=1:length(lastState.target_robots)
       targets = [targets, TargetRobot([vector(4*j-3);vector(4*j-2);vector(4*j-1);vector(4*j)],lastState.target_robots(j).rotating,lastState.target_robots(j).roombaID)];
   end
   n = 4*j +1;
   for j=1:length(lastState.obstacle_robots)
       obstacles = [obstacles, ObstacleRobot([vector(n);vector(n+1);vector(n+2)],lastState.obstacle_robots(j).paused,lastState.obstacle_robots(j).roombaID)];
       n = n+3;
   end
   state.target_robots = targets;
   state.obstacle_robots = obstacles;
end

