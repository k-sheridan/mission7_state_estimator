function [] = drawState(state, covarianceMatrix,sigmaPoints)
    clf;
    hold on;
    
    drawMean(state);
    drawCovariances(state,covarianceMatrix);
    %drawSigmaPoints(sigmaPoints);
    
    xlim([-10, 10]);
    ylim([-10, 10]);
    grid on;
    drawnow;
    hold off;

end

