logit_map = zeros(180 / dTheta + 1, 360 / dTheta + 1);

trajectory_start = (Start_Angles(elbow_start,:));
trajectory_goal = (Goal_Angles(elbow_goal,:));

for i = 1 : length(CSpace_Obstacles)
    logit_map(CSpace_Obstacles(i,1)/dTheta + 1, CSpace_Obstacles(i,2)/dTheta + 1) = 1;
end

x_axis = (0 : dTheta : 180);
y_axis = (0 : dTheta : 360);

trajectory = wavefront(trajectory_start, trajectory_goal, x_axis, y_axis, logit_map);
trajectory = trajectory * dTheta;
trajectory = [trajectory_start; trajectory ; trajectory_goal];

figH = figure('Position', [50 100 1200 500]);
sgtitle({"Wavefront Implementation",strcat("L_{1}= ",num2str(L1)," ,"," L_{2}= ",num2str(L2))});
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
    plot(trajectory(via,1), trajectory(via,2),'r.','LineWidth',1.5);
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