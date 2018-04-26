function [] = plotxy_target(data,dtotal,dt,state)

f1 = figure;
figure(f1);
clf;
hold on;

delta_xy = []
plots = [];
labels = [];
colors = distinguishable_colors(length(state.target_robots));

for column=data
    new_norms = [];
    temp_state = vectortoObject(column,state)
    for target = temp_state.target_robots
        new_norms = [new_norms;norm([target.state(1);target.state(2)])];
    end
    delta_xy = [delta_xy,new_norms];
end

r=1;
for target = temp_state.target_robots
    labels = [labels,strcat("target robot ",num2str(r))];
    r = r+1;
end

r=1;
for columns=delta_xy'
    plots = [plots,plot((dt:dt:dtotal),columns','Color',colors(r,:))];
    r = r+1;
end

lgnd = legend(plots,labels)
lgnd.FontSize = 18;
xlabel("Time (seconds)",'Fontsize',18);
ylabel("Error (meters)",'Fontsize',18);
title("Positional Error for Targets Robots Over time",'FontSize',20);
hold off;

end

