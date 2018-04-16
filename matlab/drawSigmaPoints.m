function [] = drawSigmaPoints(sigmaPoints)
    %draw simga Points
    for point=sigmaPoints
       for element=point.target_robots
            quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',1,'MaxHeadSize',0.5);
       end
    
        for element=point.obstacle_robots
            quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',1,'MaxHeadSize',0.5);
        end 
    end
end

