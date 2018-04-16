function [] = drawCovariances(state,covarianceMatrix)
%draw covariances
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
end

