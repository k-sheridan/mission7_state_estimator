function [next_state] = process(state, deltaT, time_step)

roomba_angular_velocity = 
speed = 0.33
bumperAngle = 2.44346
obstacle_angular_velocity = 0.066
radius = 1

robots = [state.target_robots, state.obstacle_robots]

for 0:time_step:deltaT
    for i=1:robots
        if i ~= length(robots)
            for j=i+1:robots
                distance = sqrt( (robots(i).state(1) - robots(j).state(1))^2 + (robots(i).state(2) - robots(j).state(2))^2 )
                if distance <= 2*radius 
                    % robot i is left of robot j
                    if robots(i).state(1) <= robots(j).state(1)
                        if pi - bumperAngle/2 <= abs(robots(i).state(3) - robots(j).state(3)) <= pi + bumperAngle/2 
                            if robots(i).state(3) - robots(j).state(3) <= 0
                               % front end collision
                               
                            else
                               
                               %no collision facing opppsite directions
                               break
                                
                            end
                            
                        else
                            
                            %rear end collision
                             if robots(i).state(3) - robots(j).state(3) <= 0
                                %robot i crashed into robot j
                            else
                                %robot j crashed into robot i
                            end
                        
                        end
                        
                     %robot i is right of robot j  
                    else
                        if pi - bumperAngle/2 <= abs(robots(i).state(3) - robots(j).state(3)) <= pi + bumperAngle/2 
                            if robots(i).state(3) - robots(j).state(3) >= 0
                               % front end collision
                               
                            else
                               
                               %no collision facing opppsite directions
                               break
                                
                            end
                            
                        else
                            %rear end collision
                            if robots(i).state(3) - robots(j).state(3) >= 0
                                %robot i crashed into robot j
                            else
                                %robot j crashed into robot i
                            end
                            
                        
                        end
                        
                    end
                end
            end
        end
    end
end

    

end

