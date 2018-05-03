function [bool] = detectCollision(roomba1,roomba2,bound)
    %detect if roomba 1 is colliding
    if strcmp(class(roomba1),'TargetRobot') == true
        if(roomba1.rotating ~= 0)
            bool = false;
            return
        end
    else
        if(roomba1.paused ~= 0)
            bool = false;
            return
        end
    end
        

    dr = roomba2.state(1:2) - roomba1.state(1:2);
    
    R = [cos(roomba1.state(3)), -sin(roomba1.state(3));
        sin(roomba1.state(3)), cos(roomba1.state(3))];
    
    b_dr = R'*dr;
    
    theta = atan2(b_dr(2), b_dr(1));
    
    if(abs(theta) < bound)
       bool = true;
    else
       bool = false;
    end
end

