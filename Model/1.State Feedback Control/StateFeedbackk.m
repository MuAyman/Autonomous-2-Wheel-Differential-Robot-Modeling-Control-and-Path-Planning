clear; clc;

%% Set points (Assuming the intrial position is at the origin)
pho_i = 100;
alpha_i = 170 * pi/180;
beta_i = -150 * pi/180;

%% These conditions ensure that all three eigenvalues of matrix A are negative.
% kp > 0
% kakp - kbkp > 0
% kp + ka > 0

kp = 5;
ka = 10;
kb = -1.2;
A = [-kp 0 0; 0 kp-ka -kb; 0 -kp 0];
eig(A)