function [] = obstacleMove(obstacle,obstacle_angular_velocity,speed,deltaT)
    magnitude = speed * deltaT
    obstacle.x = obstacle.x + cos(obstacle.theta) * magnitude
    obstacle.y = obstacle.y + sin(obstacle.theta) * magnitude
    obstacle.theta = obstacle.theta + obstacle_angular_velocity * deltaT
end

