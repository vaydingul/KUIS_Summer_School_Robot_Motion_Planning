trajectory_start = (Start_Angles(elbow_start,:)); % Set trajectory starting point
trajectory_goal = (Goal_Angles(elbow_goal,:));  % Set trajectory end point

alpha_axis = (0 : dTheta : 180); % Discretized x axis
beta_axis = (0 : dTheta : 360); % Discretized y axis
[alpha_grid, beta_grid] = meshgrid(alpha_axis, beta_axis); % Meshgrid

p0 = 10; % Repulsive distance of influence
repulsion_gain = 1000; % Repulsion gain
attraction_gain = 1; % Attraction gain

potential = calculate_potential_map(attraction_gain, repulsion_gain, p0,... % Calculate potential for every point in the grid
    trajectory_goal, CSpace_Obstacles, alpha_axis, beta_axis);
figure
hold on
plot(trajectory_goal(1,1),trajectory_goal(1,2),'*r','MarkerSize',10); % Draw 2D contour of potential
contour(alpha_grid,beta_grid,potential,60);
xlim([0 180]);
ylim([0 360]);
axis square
xlabel('\alpha');
ylabel('\beta');
legend('Goal Configuration','Location','Best');
title('Potential Field Contours');

figure
xlabel('\alpha');
ylabel('\beta');
meshc(alpha_grid,beta_grid,potential) % 3D mesh surface of potential
title('Potential Field Mesh');
shading interp

trajectory = trajectory_start; % Create trajectory array
idx = 1; 
dI_min = 0.5;
dI = dI_min;
proximity_threshold = 0.1;

fprintf('Trajectory calculation started.');
tic
while true
    if mod(idx,100) == 0
        fprintf('.');
    end
    local_x = trajectory(idx,1) - dI : dI_min : trajectory(idx,1) + dI;
    local_y = trajectory(idx,2) - dI : dI_min : trajectory(idx,2) + dI;
    
    [grid_x, grid_y] = meshgrid(local_x, local_y);
    local_grid = [grid_x(:) grid_y(:)];
    
    [k,dist] = dsearchn(CSpace_Obstacles, local_grid);
    dist(dist > p0) = p0;
    
    repulsive_potentials = 0.5 * repulsion_gain * ( (1 ./ dist) - (1 / p0) ) .^ 2;
    
    attraction_distances = sum((local_grid - trajectory_goal) .^ 2, 2);
    attractive_potentials = 0.5 * attraction_gain * attraction_distances;
    
    potentials = attractive_potentials + repulsive_potentials;
    
    [~, idx_of_min_potential] = min(potentials);
    
    min_potential_point = local_grid(idx_of_min_potential, :);
    
    trajectory = [trajectory ; min_potential_point];
    idx = idx + 1;
    
    if pdist([trajectory(end,:); trajectory_goal]) <= proximity_threshold || dI >= 3
        break;
    end
    
    if isequal(trajectory(end,:), trajectory(end-1,:))
        trajectory = trajectory(1 : end - 1,:);
        idx = idx - 1;
        dI = dI + dI_min;
    else
        dI = dI_min;
    end
end
elapsed_time = toc;
fprintf('\n%g trajectory points generated in %.2f seconds.\n', idx, elapsed_time);

figH = figure('Position', [50 100 1200 500]);
sgtitle({"Potential Field Implementation",strcat("L_{1}= ",num2str(L1)," ,"," L_{2}= ",num2str(L2))});
subplot(1,2,1);
hold on
grid on
circle(obstacle_x, obstacle_y, obstacle_r); % Draws a circle
p1 = plot(start_x, start_y,'*g','LineWidth',2,'MarkerSize',10); % Mark one of the starting configurations
p2 = plot(goal_x, goal_y,'*r','LineWidth',2,'MarkerSize',10); % Mark one of the goal configurations
legend([p1 p2],'Starting Coordinate','Commanded Coordinate','Location','SouthEast','AutoUpdate','off')

xlim([0 100]);
ylim([0 100]);
xlabel('x');
ylabel('y');
title('Cartesian Space');

subplot(1,2,2);
hold on
grid on
plot(trajectory(1,1),trajectory(1,2),'b*','MarkerSize',10)
plot(trajectory(end,1),trajectory(end,2),'r*','MarkerSize',8)
plot(trajectory_goal(1,1),trajectory_goal(1,2),'go','LineWidth',1.5,'MarkerSize',10)
plot(CSpace_Obstacles(:,1),CSpace_Obstacles(:,2),'k.'); % Plot the obstacles
contour(alpha_grid,beta_grid,potential,60);
xlim([0 180]);
ylim([0 360]);
xlabel('\alpha');
ylabel('\beta');
title("Configuration Space");
legend('First Point On The Trajectory','Last Point On The Trajectory','Commanded Configuration','Location','SouthEast','AutoUpdate','off')

if record_video
    video_name = strcat(mat2str(use_wavefront),'_',...
        num2str(round(trajectory_start(1),1)),'-',num2str(round(trajectory_start(2),1)),'_',...
        num2str(round(trajectory_goal(1),1)),'-',num2str(round(trajectory_goal(2),1)));
    myVideo = VideoWriter(video_name);
    myVideo.FrameRate = 20;
    open(myVideo)
end

for via = 1 : length(trajectory)
    
    a = trajectory(via,1);
    b = trajectory(via,2);
    
    x1 = L1 * cosd(a) + 20;
    x1 = linspace(20,x1,4);
    
    y1 = L1 * sind(a);
    y1 = linspace(0,y1,4);
    
    x2 = L1 * cosd(a) + L2 * cosd(a + b) + 20;
    x2 = linspace(x1(end),x2,4);
    
    y2 = L1 * sind(a) + L2 * sind(a + b);
    y2 = linspace(y1(end),y2,4);
    
    if ~ishghandle(figH)
        break
    end
    
    subplot(1,2,1);
    t = text(5,95,strcat("Trajectory progress: ",num2str(100 * via / length(trajectory)),"%"));
    link1 = line(x1,y1,'Color','green','LineWidth',2);
    link2 = line(x2,y2,'Color','blue','LineWidth',2);
    drawnow
    
    subplot(1,2,2);
    hold on
    pth = plot(trajectory(1:via,1), trajectory(1:via,2),'r-','LineWidth',1.5,'MarkerSize',2);
    drawnow
    
    if record_video
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
    end
    
    if via ~= length(trajectory)
        delete(t);
        delete(link1);
        delete(link2);
    end
end
if record_video
    close(myVideo);
end
fprintf('End of animation.\n');

function h = circle(center_x, center_y,r)
hold on
angle = linspace(0, 2 * pi, 1000);
xunit = r * cos(angle) + center_x;
yunit = r * sin(angle) + center_y;
h = plot(xunit, yunit);
fill(xunit, yunit, 'r');
end
