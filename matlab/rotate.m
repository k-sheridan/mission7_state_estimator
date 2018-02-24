function [] = rotate(roomba,deltaT,angularVelocity)
    roomba.rotating = roomba.rotating - deltaT
    roomba.theta = roomba.theta + angularVelocity(deltaT)
end

