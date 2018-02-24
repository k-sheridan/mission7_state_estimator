function [] = drawState(state)

    hold on;
    
    for element=state.target_robots
        quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',2)
    end
    
    for element=state.obstacle_robots
        quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',2)
    end
    
    xlim([-10, 10]);
    ylim([-10, 10]);
    grid on;
end

