function [obstacle] = obstacleMove(obstacle,obstacle_angular_velocity,speed,deltaT)
    magnitude = speed * deltaT
    obstacle.state(1) = obstacle.state(1) + cos(obstacle.state(3)) * magnitude
    obstacle.state(2) = obstacle.state(2) + sin(obstacle.state(3)) * magnitude
    obstacle.state(3) = obstacle.state(3) + obstacle_angular_velocity * deltaT
end

