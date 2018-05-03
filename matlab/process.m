function [next_state] = process(state, deltaT,time_step)

roomba_angular_velocity = -pi/2;
speed = 0.33;
%bumber arc
bumperAngle = 2.44346;
obstacle_angular_velocity = -0.066;
radius = 0.17018;

robots = [state.target_robots, state.obstacle_robots];

%for loop doesn't work
for t=time_step:time_step:deltaT
    %check for collisions
    for i=1:length(robots)
        if i ~= length(robots)
            for j=i+1:length(robots)
                distance = norm(robots(i).state(1:2, 1) - robots(j).state(1:2, 1));
                if distance <= 2*radius
                    

                    if detectCollision(robots(i),robots(j),pi/2) == true && robots(i).collidedThisTimeStep == false 
                        robots(i).collidedThisTimeStep = true;
                        if strcmp(class(robots(i)),'TargetRobot') == true
                           robots(i).rotating = robots(i).rotating + -pi/roomba_angular_velocity;
                           state.target_robots(i) = robots(i);
                        else
                            robots(i).paused = robots(i).paused + -pi/roomba_angular_velocity;
                            state.obstacle_robots(i - length(state.target_robots)) = robots(i);
                        end
                    end
                    if detectCollision(robots(j),robots(i),pi/2) == true && robots(j).collidedThisTimeStep == false 
                        robots(j).collidedThisTimeStep = true;
                        if strcmp(class(robots(j)),'TargetRobot') == true
                            robots(j).rotating = robots(j).rotating + -pi/roomba_angular_velocity;
                            state.target_robots(j) = robots(j);
                        else
                            robots(j).paused = robots(j).paused + -pi/roomba_angular_velocity;
                            state.obstacle_robots(j - length(state.target_robots)) = robots(j);
                        end
                    end
                end
            end
        end
    end
    for reset=1:length(robots)
        robots(reset).collidedThisTimeStep = false;
        % reset rotating of rotating is less than 0. rotating should never
        % be less than 0!
        if strcmp(class(robots(reset)),'TargetRobot') == true
            if robots(reset).rotating < 0
                robots(reset).rotating=0;
            end
            %add to the robot's internal timer
            robots(reset).state(4) = robots(reset).state(4) + time_step;
            %don't worry about this
            state.target_robots(reset) = robots(reset);
        else
            % reset paused of paused is less than 0. rotating should never
            % be less than 0!
            if robots(reset).paused < 0
                robots(reset).paused=0;
            end
            state.obstacle_robots(reset - length(state.target_robots)) = robots(reset);
        end
    end
    
    
    %update target robots
    for target=1:length(state.target_robots)
        %if their internal timer is negative do nothing
        if state.target_robots(target).state(4) < 0
            continue;
        end
        %if they are out of bounds do nothing
        if state.target_robots(target).state(1) >= 10 || state.target_robots(target).state(2) >=10 || state.target_robots(target).state(2) <= -10 || state.target_robots(target).state(1) <= -10 
            continue;
        end
        %if 20 seconds passed rotate
        if state.target_robots(target).state(4) >= 20 && mod(state.target_robots(target).state(4),20) <= time_step
            state.target_robots(target).rotating = state.target_robots(target).rotating + -pi/roomba_angular_velocity;
        end
        %if the roomba is supposed to be rotating then rotate it.
        if state.target_robots(target).rotating ~= 0
             state.target_robots(target) = rotateRoomba(state.target_robots(target),time_step,roomba_angular_velocity);
             continue;
        end
        state.target_robots(target)= moveForward(state.target_robots(target),speed,time_step);
    end
    %update obstacle
    for obstacle=1:length(state.obstacle_robots)
        %if the robot is supposed to be paused then pause.
        if state.obstacle_robots(obstacle).paused ~= 0
             state.obstacle_robots(obstacle) = obstaclePause(state.obstacle_robots(obstacle),time_step);
             continue;
        end
        state.obstacle_robots(obstacle)= obstacleMove(state.obstacle_robots(obstacle),obstacle_angular_velocity,speed,time_step);
    end
    

end


next_state = state;

end

