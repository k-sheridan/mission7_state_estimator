function [] = plotTimer(data,dtotal,dt,state)

f5 = figure;
figure(f5);

clf;
hold on;

delta_timer = [];
plots = [];
labels = [];
colors = distinguishable_colors(length(state.target_robots));

for column=data
    new_norms = [];
    temp_state = vectortoObject(column,state);
    for target = temp_state.target_robots
        new_norms = [new_norms;target.state(4)];
    end
    delta_timer = [delta_timer,new_norms];
end

r =1;
for target = temp_state.obstacle_robots
    labels = [labels,strcat("target robot ",num2str(r))];
    r = r+1;
end

r=1;
for columns=delta_timer'
    plots = [plots,plot((dt:dt:dtotal),columns','Color',colors(r,:))];
    r = r+1;
end

lgnd = legend(plots,labels)
lgnd.FontSize = 18;
xlabel("Time (seconds)",'Fontsize',18);
ylabel("Error (seconds)",'Fontsize',18);
title("Internal Timer Error for Targets Robots Over time",'FontSize',20);
hold off;

end

