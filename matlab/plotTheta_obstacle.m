function [] = plotTheta_obstacle(data,dtotal,dt,state)

f4 = figure;
figure(f4);

clf;
hold on;

delta_theta = [];
plots = [];
labels = [];
colors = distinguishable_colors(length(state.obstacle_robots));

for column=data
    new_norms = [];
    temp_state = vectortoObject(column,state);
    for obstacle = temp_state.obstacle_robots
        new_norms = [new_norms;obstacle.state(3)];
    end
    delta_theta = [delta_theta,new_norms];
end

r =1;
for obstacle = temp_state.obstacle_robots
    labels = [labels,strcat("obstacle robot ",num2str(r))];
    r = r+1;
end

r=1;
for columns=delta_theta'
    plots = [plots,plot((dt:dt:dtotal),columns','Color',colors(r,:))];
    r = r+1;
end

lgnd = legend(plots,labels)
lgnd.FontSize = 18;
xlabel("Time (seconds)",'Fontsize',18);
ylabel("Error (radians)",'Fontsize',18);
title("Angular Error for Obstacles Robots Over time",'FontSize',20);
hold off;

end
