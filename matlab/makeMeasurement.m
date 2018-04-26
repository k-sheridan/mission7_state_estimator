function [measurement] = makeMeasurement(state,variance)
    measurement = [];
    for target=state.target_robots
       measurement = [measurement; target.state(1) + variance*randn];
       measurement = [measurement; target.state(2) + variance*randn];
    end
    for obstacle = state.obstacle_robots
       measurement = [measurement; obstacle.state(1) + variance*randn];
       measurement = [measurement; obstacle.state(2) + variance*randn];
    end
end

