function [roomba] = rotate(roomba,deltaT,angularVelocity)
    roomba.rotating = roomba.rotating - deltaT
    roomba.state(3) = roomba.state(3) + angularVelocity*deltaT
end

