function visualize_trajectories_3D(map3D, Trajectories, ttle)
% Trajectories is a cell array of trajectories

ntraj = length(Trajectories);

fig = figure
show(map3D)
axis equal
view([40 -15])
hold on

% Create the scatter plot
for i = 1:ntraj
    traj = Trajectories{i};
    scatter3(traj(1,1),traj(1,2),traj(1,3),"r","filled")
    scatter3(traj(end,1),traj(end,2),traj(end,3),"blue","filled")
    x3 = traj(:, 1);
    y3 = traj(:, 2);
    z3 = traj(:, 3);
    plot3(x3, y3, z3, "r-",LineWidth=2);
end

% Add a title with a smaller font size
title(ttle, 'FontSize', 10, 'FontWeight', 'bold');

% Add x, y, z labels with smaller font size
xlabel('x (m)', 'FontSize', 9);
ylabel('y (m)', 'FontSize', 9);
zlabel('z (m)', 'FontSize', 9);

% Add grid and adjust font size for axes
grid on;
set(gca, 'FontSize', 8); % Tick labels and other axes text

% Adjust figure size for two-column layout
set(gcf, 'Units', 'inches', 'Position', [0, 0, 3.5, 3]); % Width = 3.5 inches for one column

hold off;

saveas(fig, strcat(ttle,'.fig')); % Save as PNG

end

% show(map3D)
% axis equal
% view([10 15])
% hold on
% % Start state
% scatter3(start(1,1),start(1,2),start(1,3),"r","filled")
% % Goal state
% scatter3(goal(1,1),goal(1,2),goal(1,3),"g","filled")
% % Path
% plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3),"r-",LineWidth=2)
% 
