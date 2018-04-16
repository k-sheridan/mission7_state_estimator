function [] = drawMean(state)
%draw state
    for element=state.target_robots
        quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',2,'MaxHeadSize',1);
    end
    
    for element=state.obstacle_robots
        quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',2,'MaxHeadSize',1);
    end
end

