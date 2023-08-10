clc; close all;
%% Initialization
% Setpoints
startPos = [0; 0; 0]; % Initial robot position [x, y, theta]
goalPos = [5; 5; 0]; % Goal position [x, y, theta]
obstaclePos = [2, 3, 1; 3, 4, 1; 2, 3, 1; 3, 4, 0.8; 4, 2, 0.5]'; % [x; y; radius] 3*N
timeStep = 0.1; % Time step

% Lyapunov parameters
Kx = 5;
Kt = 10;

%% Path planing function
[Xref,Yref,Thetaref] = APFPathPlanning(startPos,goalPos,obstaclePos,timeStep);

%% Evaluating A,B,&C matrices
syms x y theta V w
f = [V*cos(theta); V*sin(theta); w];
states = [x y theta];
inputs = [V w];
A = jacobian(f,states);
B = jacobian(f,inputs);
C = eye(3);

%% Extended Kalman Filter Parameters
SysQ = 0.01;    % system states covariance
NoisePower = 0.01;

