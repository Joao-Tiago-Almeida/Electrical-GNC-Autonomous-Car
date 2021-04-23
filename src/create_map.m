close all;
clear;
clc;

%% get window dimensions
% f1=figure(1);
% pos_normal = f1.Position(3:4);
% f1.WindowState = 'maximized';
% pause(0.5); % waiting for the window to expand
% pos_fullscreen = f1.Position(3:4);
% close 1

%% display images
MAP = figure('Name','MAP','NumberTitle','off');
pbaspect([1 1 1]);
I = imread('../Maps images/IST_campus.png');
MAP.Children.Position = [0 0 1 1];
imshow(I);
MAP.WindowStyle = 'docked';
% MAP.MenuBar = 'none';
% MAP.ToolBar = 'none';
% len = pos_normal(1) + pos_fullscreen(2)-pos_normal(2);
% MAP.Position(3:4) = [len len];
hold on;
%% set MAP scale
[meters_from_MAP, fget_Lat_from_MAP, fget_Lon_from_MAP] = scale_map(size(I,1));
