function [stateVector] = objecttoVector(state)
    stateVector = [];
    for target=state.target_robots
        stateVector = [vector; target.state.'];
    end 
    for obstacle=state.obstacle_robots
        stateVector = [vector; obstacle.state.'];
    end
end
