clear; clc; close all;

% Initialization
startPos = [0; 0]; % Initial robot position [x, y]
goalPos = [5; 3]; % Goal position [x, y]
obstaclePos = [2, 3, 1; 3, 4, 1; 2, 3, 1; 3, 4, 0.8; 4, 2, 0.5]'; % [x, y, radius]
timeStep = 0.1; % Time step

robotPath = APFPathPlanning(startPos,goalPos,obstaclePos,timeStep);