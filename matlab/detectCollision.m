function [bool] = detectCollision(roomba1,roomba2,min,max)
    world_frame_coords = [roomba1.state(1) - roomba2.state(1); roomba1.state(2) - roomba2.state(2)]
    rotation_matrix = [cos(roomba1.state(3)) sin(roomba1.state(3)); -sin(roomba1.state(3)) cos(roomba1.state(3))]
    transformed_matrix = rotation_matrix * world_frame_coords
    theta = atan2(transformed_matrix(2),transformed_matrix(1));
    if min < theta && theta < max
        bool = true
    else
        bool = false
    end
    
end

