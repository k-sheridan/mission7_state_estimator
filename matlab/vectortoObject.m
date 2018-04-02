function [state] = vectortoObject(vector,lastState)
   targets= [];
   obstacles = [];
   state = State;
   size(vector)
   for j=1:length(lastState.target_robots)
       targets = [targets, TargetRobot([vector(4*j-3),vector(4*j-2),vector(4*j-1),vector(4*j)],lastState.target_robots(j).rotating,lastState.target_robots(j).roombaID)];
   end
   for j=j+1:j+length(lastState.obstacle_robots):3
       obstacles = [obstacles, ObstacleRobot([vector(j-2),vector(j-1),vector(j)],lastState.target_robots(j).paused,lastState.target_robots(j).roombaID)];
   end
   state.target_robots = targets;
   state.obstacle_robots = obstacles;
end

