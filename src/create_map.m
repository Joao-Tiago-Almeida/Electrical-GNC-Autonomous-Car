function [occupancyMatrix, pathPoints, MAP_info, safe_matrix] = create_map(path_img)
if nargin < 1
    path_img = '../Maps images/IST_campus.png';
    MAP_info = load('../mat_files/mapInformation.mat');
else
    MAP_info = scale_map();
end
%% get window dimensions
% f1=figure(1);
% pos_normal = f1.Position(3:4);
% f1.WindowState = 'maximized';
% pause(0.5); % waiting for the window to expand
% pos_fullscreen = f1.Position(3:4);
% close 1

%% display images
MAP = figure('Name','MAP','NumberTitle','off'); % figure 1! do not change
pbaspect([1 1 1]);
I = imread(path_img);
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

%% Define the roads by drawing polygons
[X,Y] = meshgrid(1:size(I, 2),1:size(I, 1));
occupancyMatrix = zeros(size(I, 1), size(I, 2));
while true
    roadMarkers = drawRoads();
    occupancyMatrix = occupancyMatrix + inpolygon(X, Y, roadMarkers(:,1)', roadMarkers(:,2)' );
    
    if ( strcmp(input("\nIf you're done drawing this road enter 'Y', if not press 'N': ", 's'), 'Y') == 1 )
        break;
    end
end

occupancyMatrix(occupancyMatrix>0) = 1; %Normalizing to 1 because 2 roads may be overlapped

%% Now defining other specificities in the environment (defined in binary matrix)

% Crosswalk defined in binaryMatrix with 2's
counter = 0;
while true
    if counter==0
        if ( strcmp(input("\nIf you want to draw a CrossWalk press 'Y', if not press 'N': ", 's'), 'N') == 1 )
            break;
        end
        counter = 1;
    end
    crossWalk = drawCrosswalk();
    occupancyMatrix(logical(inpolygon(X, Y, crossWalk(:,1)', crossWalk(:,2)' ) .* occupancyMatrix)) = 2;
    
    if ( strcmp(input("\nIf you want to draw more CrossWalks press 'Y', if not press 'N': ", 's'), 'N') == 1 )
        break;
    end
end

% TrafficLights defined in binaryMatrix with 3's
counter = 0;
while true
    if counter==0
        if ( strcmp(input("\nIf you want to draw Traffic Lights press 'Y', if not press 'N': ", 's'), 'N') == 1 )
            break;
        end
        counter = 1;
    end
    trafficLights = drawTrafficLight();
    occupancyMatrix(logical(inpolygon(X, Y, trafficLights(:,1)', trafficLights(:,2)' ) .* occupancyMatrix)) = 3;
    
    if ( strcmp(input("\nIf you want to draw more traffic lights press 'Y', if not press 'N': ", 's'), 'N') == 1 )
        break;
    end
end

% Stop signs defined in binaryMatrix with 4's
counter = 0;
while true
    if counter==0
        if ( strcmp(input("\nIf you want to draw Stop Signs press 'Y', if not press 'N': ", 's'), 'N') == 1 )
            break;
        end
        counter = 1;
    end
    stopSign = drawStopSign();
    occupancyMatrix(logical(inpolygon(X, Y, stopSign(:,1)', stopSign(:,2)' ) .* occupancyMatrix)) = 4;
    if ( strcmp(input("\nIf you want to draw Stop Signs press 'Y', if not press 'N': ", 's'), 'N') == 1 )
            break;
    end
end

save('../mat_files/MAP.mat', 'MAP');
save('../mat_files/occupancyMatrix.mat', 'occupancyMatrix');
Image = getframe(gcf);
pause(0.2)
imwrite(Image.cdata, '../mat_files/MAP_w_roads.png');

%% Picking the start and end points and the intermediate ones
pathPoints = pickPathPoints();

%% get safety matrix

% supostly read from a global file
safe_distance = 1.5;    % meters
forbidden_zone = 0.64;  % meters
safe_matrix = draw_safe_matrix(occupancyMatrix, MAP_info.meters_from_MAP, safe_distance, forbidden_zone);

%% draw speacial regions on the map

figure;
mesh(occupancyMatrix)
colorTheme = [ 0 0 0
128 128 128
255 255 255
0 255 0
255 0 0
]/255;
colormap(colorTheme);