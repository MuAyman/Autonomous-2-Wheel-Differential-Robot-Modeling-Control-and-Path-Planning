function robotPath = APFPathPlanning(startPos,goalPos,obstaclePos,timeStep)
% Potential field parameters
k_att_min = 1; % Minimum attractive force gain
k_att_max = 10; % Maximum attractive force gain
k_att_distance = 0.5; % Distance at which attractive force is maximum
k_rep = 8.7; % Repulsive force gain
repulsive_range = 1.5; % Repulsive range of obstacles
virtual_obstacle_radius = 0.2; % Radius of the virtual obstacle

% Initialize robot path
robotPos = startPos;
robotPath = robotPos;

% Create figure and axes
figure(1);
ax = gca;
ax.NextPlot = 'replaceChildren';

% Simulation loop
timeTaken = 0; prev = 1;
while norm(robotPos - goalPos) > 0.05
    % Calculate distance between robot and goal
    distance = norm(robotPos - goalPos);

    % Calculate the gradually increasing attractive force
    k_att = k_att_max + (k_att_min - k_att_max) * (1 - exp(-distance / k_att_distance));
    attractive_force = k_att * (goalPos - robotPos);

    % Calculate repulsive force
    repulsive_force = [0; 0];
    for i = 1:size(obstaclePos, 2)
        obstacle = obstaclePos(:, i);
        obstacle_dist = norm(robotPos - obstacle(1:2));
        if obstacle_dist < repulsive_range + obstacle(3)
            repulsive_force = repulsive_force + k_rep * ((1 / obstacle_dist) - (1 / (repulsive_range + obstacle(3)))) * ((robotPos - obstacle(1:2)) / obstacle_dist^2);
        end
    end

    % Calculate total force
    total_force = attractive_force + repulsive_force;

    % Update robot position
    robotPos = robotPos + total_force * timeStep;

    % Add current position to robot path
    robotPath = [robotPath, robotPos];

    % check if stuck in local minima
    if prev <= 5
        PosPrev(:,prev) = robotPos;
        if prev == 5
            if  norm(PosPrev(:,5) -  PosPrev(:,1)) < 0.01
                % check if stuck near the goal (Don't add virtual obstacle)
                if norm(goalPos - robotPos) < 0.5
                    disp("timeTaken = "); disp(timeTaken);
                    break
                end
                disp("Stuck in local minima. Adding virtual obstacle.");
                virtual_obstacle_pos = [robotPos(1); robotPos(2); virtual_obstacle_radius];
                obstaclePos = [obstaclePos, virtual_obstacle_pos];
                APFPathPlanning(startPos,goalPos,obstaclePos,timeStep);
                break
            end
        end
        prev = prev + 1;
    else
        prev = 1;
    end


    % Plot robot and goal positions with path
    plot(robotPath(1, :), robotPath(2, :), 'b-', 'LineWidth', 2);
    hold on;
    plot(robotPos(1), robotPos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    hold on;
    plot(goalPos(1), goalPos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    hold on;
    plotObstacles(obstaclePos);
    hold off;
    xlim([min(goalPos(1),startPos(1))-1, max(goalPos(1),startPos(1))+1]);
    ylim([min(goalPos(2),startPos(2))-1, max(goalPos(2),startPos(2))+1]);
    grid on;
    title('APF Path Planning');
    legend('Robot Path', 'Robot', 'Goal', 'Location', 'southoutside', 'Orientation', 'horizontal');

    % Pause for visualization
    pause(timeStep);

    % Check if the robot has reached the goal
    if norm(robotPos - goalPos) < 0.05
        disp('Goal reached!');
        disp("timeTaken = "); disp(timeTaken);
    end

    timeTaken = timeTaken + timeStep;
end
end

function plotObstacles(obstaclePos)
numObstacles = size(obstaclePos, 2);
for i = 1:numObstacles
    obstacle = obstaclePos(:, i);
    x = obstacle(1) + obstacle(3) * cos(linspace(0, 2*pi, 100));
    y = obstacle(2) + obstacle(3) * sin(linspace(0, 2*pi, 100));
    plot(x, y, 'k-', 'LineWidth', 2);
    hold on;
end
end


