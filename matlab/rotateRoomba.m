function [roomba] = rotateRoomba(roomba,deltaT,angularVelocity)
    roomba.rotating = roomba.rotating - deltaT; %delta T is not the same as delta T in the process!ß
    roomba.state(3) = roomba.state(3) + angularVelocity*deltaT;
end

