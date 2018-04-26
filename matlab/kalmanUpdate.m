function [updateState,newCovariance] = kalmanUpdate(state,covariance,H,measurement,measurement_uncertainty)

% calculate R 
R = diag(ones(1, 28)*measurement_uncertainty);

state_vector = objecttoVector(state);

%error
y = measurement - H*state_vector



S = H*covariance*H' + R;

%Kalman gain
K = covariance * H' * inv(S);
updateState = state_vector + K * y;
updateState = vectortoObject(updateState,state);
newCovariance = (eye(size(covariance,1)) - K * H) * covariance * (eye(size(covariance,1)) - K * H)' + K*R*K';

u = measurement - H * objecttoVector(updateState)

end

