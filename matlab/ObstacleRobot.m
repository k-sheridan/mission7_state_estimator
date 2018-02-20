classdef ObstacleRobot
    
    properties
        state; %[x, y, theta]
        paused; % Delta T value reflecting how long the obstacle is paused. If its 0 then its not paused 
    end
    
    methods
        function obj = ObstacleRobot(current_state, paused)
            obj.state = current_state;
            obj.paused = paused;
        end
    end
end

