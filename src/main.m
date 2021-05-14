clear all;close all;clc;
%%  GUIDANCE
% This Area is restricted to the GUIDANCE team.
% Only authorized person are admissible to change this content

% Roads-1,CrossWalks-2;TrafficLigths-3;Stop-4
load('../G_files/occupancyMatrix.mat', 'occupancyMatrix');
% Path planning points
load('../G_files/run_points.mat', 'run_points');
% Safetiness of walking in the street -  
load('../G_files/safe_matrix.mat', 'safe_matrix')
% Desired (check) Points
load('../G_files/pathPoints_grid.mat', 'points');

% Plotting an ideia of how powerful the guidance job is
MAP = load('../G_files/MAP.mat');
hold on
plot(points(:,1),points(:,2),"ko","LineWidth",5)
plot(run_points(:,1),run_points(:,2),"LineWidth",4)

%% Be my guess to continue