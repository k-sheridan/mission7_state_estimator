function [stateVector] = objecttoVector(state)
    stateVector = [];
    targets = 1;
    for target=state.target_robots
        stateVector = [stateVector; target.state];
    end 
    for obstacle=state.obstacle_robots
        stateVector = [stateVector; obstacle.state];
    end
end
