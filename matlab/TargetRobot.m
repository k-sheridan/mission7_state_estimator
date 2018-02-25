classdef TargetRobot < Roomba
    
    properties
        state; % [x, y, theta, timer]
        rotating; % delta T value reflecting how long it takes for the robot to complete rotatation. Not rotation if set to 0
        collidedThisTimeStep = false;
    end
    
    methods
        function obj = TargetRobot(current_state, rotate)
            obj.state = current_state;
            obj.rotating = rotate;
        end
    end
end

