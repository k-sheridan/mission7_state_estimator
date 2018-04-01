function [] = drawState(state, covarianceMatrix)
    clf;
    hold on;
    
    for element=state.target_robots
        quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',2,'MaxHeadSize',1);
    end
    
    for element=state.obstacle_robots
        quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',2,'MaxHeadSize',1);
    end
    
    for i=1:length(state.target_robots)+length(state.obstacle_robots)
        if i <= length(state.target_robots)
            n = 1 + 4*(i-1);
            plotGaussianEllipsoid(covarianceMatrix(n:n+1,n:n+1),[state.target_robots(i).state(1), state.target_robots(i).state(2)])
        elseif i == length(state.target_robots) + 1
            n = 1 + 4*(i-1);
            covarianceMatrix(n:n+1,n:n+1)
            plotGaussianEllipsoid(covarianceMatrix(n:n+1,n:n+1),[state.obstacle_robots(1).state(1), state.obstacle_robots(1).state(2)])
        else
            o = i - length(state.target_robots);
            n = 1+4*(length(state.target_robots))+3*(i-(length(state.target_robots) + 1));
            plotGaussianEllipsoid(covarianceMatrix(n:n+1,n:n+1),[state.obstacle_robots(o).state(1), state.obstacle_robots(o).state(2)])
        end
        
    end
    
    xlim([-10, 10]);
    ylim([-10, 10]);
    grid on;
    drawnow;
    hold off;

end

