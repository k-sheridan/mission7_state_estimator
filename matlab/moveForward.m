function [roomba] = moveForward(roomba,speed,deltaT)
    magnitude = deltaT * speed;
    roomba.state(1) = roomba.state(1) + cos(roomba.state(3)) * magnitude;
    roomba.state(2) = roomba.state(2) + sin(roomba.state(3)) * magnitude;
end
