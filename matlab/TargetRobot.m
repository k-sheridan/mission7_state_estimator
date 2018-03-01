classdef TargetRobot < Roomba
    
    properties
        state; % [x, y, theta, timer]
        rotating; % delta T value reflecting how long it takes for the robot to complete rotatation. Not rotation if set to 0
        collidedThisTimeStep = false;
        roombaID;
    end
    
    methods
        function obj = TargetRobot(current_state, rotate,id)
            obj.state = current_state;
            obj.rotating = rotate;
            obj.roombaID=id;
        end
    end
end

