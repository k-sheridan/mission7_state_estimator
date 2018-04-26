function [] = plotTheta_targets(data,dtotal,dt,state)

f3 = figure;
figure(f3);

clf;
hold on;

delta_theta = [];
plots = [];
labels = [];
colors = distinguishable_colors(length(state.target_robots));

for column=data
    new_norms = [];
    temp_state = vectortoObject(column,state);
    for target = temp_state.target_robots
        new_norms = [new_norms;target.state(3)];
    end
    delta_theta = [delta_theta,new_norms];
end

r =1;
for target = temp_state.obstacle_robots
    labels = [labels,strcat("target robot ",num2str(r))];
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
title("Angular Error for Targets Robots Over time",'FontSize',20);
hold off;

end

