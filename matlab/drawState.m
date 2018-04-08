function [] = drawState(state, covarianceMatrix,sigmaPoints)
    clf;
    hold on;
    
    for element=state.target_robots
        quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',2,'MaxHeadSize',1);
    end
    
    for element=state.obstacle_robots
        quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',2,'MaxHeadSize',1);
    end
    
    for j=1:length(state.target_robots)+length(state.obstacle_robots)
        if j <= length(state.target_robots)
            n = 1 + 4*(j-1);
            plotGaussianEllipsoid(covarianceMatrix(n:n+1,n:n+1),[state.target_robots(j).state(1), state.target_robots(j).state(2)])
        elseif j == length(state.target_robots) + 1
            n = 1 + 4*(j-1);
            plotGaussianEllipsoid(covarianceMatrix(n:n+1,n:n+1),[state.obstacle_robots(1).state(1), state.obstacle_robots(1).state(2)])
        else
            o = j - length(state.target_robots);
            n = 1+4*(length(state.target_robots))+3*(j-(length(state.target_robots) + 1));
            plotGaussianEllipsoid(covarianceMatrix(n:n+1,n:n+1),[state.obstacle_robots(o).state(1), state.obstacle_robots(o).state(2)])
        end
        
    end
    for point=sigmaPoints
       for element=point.target_robots
            quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',1,'MaxHeadSize',0.5);
       end
    
        for element=point.obstacle_robots
            quiver(element.state(1),element.state(2),cos(element.state(3)),sin(element.state(3)),'linewidth',1,'MaxHeadSize',0.5);
        end 
    end
    
    xlim([-10, 10]);
    ylim([-10, 10]);
    grid on;
    drawnow;
    hold off;

end

