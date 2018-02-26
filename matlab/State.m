classdef State
    %
    
    properties
        quad_state; %COLUMN x, y, z, r, p, y, dx, dy, dz, wx, wy, wz, ax, ay, az
        target_robots; %VECTOR of target robot state 
        obstacle_robots; %VECTOR of obstacle robots
    end
    
    methods
        function obj = State()
            obj.quad_state = zeros(15, 1);
            
        end
    end
end

