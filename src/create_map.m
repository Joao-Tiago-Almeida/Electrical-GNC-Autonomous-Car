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
MAP.Units = 'pixel';
% MAP.MenuBar = 'none';
% MAP.ToolBar = 'none';
% len = pos_normal(1) + pos_fullscreen(2)-pos_normal(2);
% MAP.Position(3:4) = [len len];
hold on;
%% set MAP scale
[MAP_info] = scale_map(size(I,1));

%% Define the roads by drawing polygons
[X,Y] = meshgrid(1:size(I, 2),1:size(I, 1));
binaryMatrix = zeros(size(I, 1), size(I, 2));
while true
    roadMarkers = drawRoads();
    binaryMatrix = binaryMatrix + inpolygon(X, Y, roadMarkers(:,1)', roadMarkers(:,2)' );
    
    if ( strcmp(input("\nIf you're done drawing this road enter 'Y', if not press 'N': ", 's'), 'Y') == 1 )
        break;
    end
end

binaryMatrix(binaryMatrix>0) = 1; %Normalizing to 1 because 2 roads may be overlapped

%% Now defining other specificities in the environment (defined in binary matrix)

% Crosswalk defined in binaryMatrix with 2's
while true
    crossWalk = drawCrosswalk();
    binaryMatrix(logical(inpolygon(X, Y, crossWalk(:,1)', crossWalk(:,2)' ) .* binaryMatrix)) = 2;
    
    if ( strcmp(input("\nIf you're done drawing this crosswalk enter 'Y', if not press 'N': ", 's'), 'Y') == 1 )
        break;
    end
end

% TrafficLights defined in binaryMatrix with 3's
while true
    trafficLights = drawTrafficLight();
    binaryMatrix(logical(inpolygon(X, Y, trafficLights(:,1)', trafficLights(:,2)' ) .* binaryMatrix)) = 3;
    
    if ( strcmp(input("\nIf you're done drawing this traffic light enter 'Y', if not press 'N': ", 's'), 'Y') == 1 )
        break;
    end
end

% Stop signs defined in binaryMatrix with 4's
while true
    stopSign = drawStopSign();
    binaryMatrix(logical(inpolygon(X, Y, stopSign(:,1)', stopSign(:,2)' ) .* binaryMatrix)) = 4;
    
    if ( strcmp(input("\nIf you're done drawing this stop sign enter 'Y', if not press 'N': ", 's'), 'Y') == 1 )
        break;
    end
end

%% Picking the start and end points and the intermediate ones
pathPoints = pickPathPoints(binaryMatrix);

save('binaryMatrix.mat', 'binaryMatrix');
save('pathPoints.mat', 'pathPoints');

figure()
mesh(flip(binaryMatrix))
colorTheme = [ 0 0 0
128 128 128
255 255 255
0 255 0
255 0 0
]/255;
colormap(colorTheme);