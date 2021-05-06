clear;
close all;
clc;

% TODO link - https://notability.com/n/0goKZZ3B87j8BrKcE3ruIi

%%
disp('WELCOME TO THE GRAPHICAL GUIDANCE USER INTERFACE')
using_default_configurations = input('Do you want to change the default configurations? y/n\n', 's');
if(strcmp(using_default_configurations, 'y'))
    if(strcmp(input('Do you want to change the map? y/n\n', 's'), 'y'))
        path_img = input('Input new map path', 's');
        [occupancyMatrix, pathPoints, MAP_info, safe_matrix] = create_map(path_img);
    else
        if(strcmp(input('Do you want to change the road? y/n\n', 's'), 'y'))
            [occupancyMatrix, pathPoints, MAP_info, safe_matrix] = create_map();
        else
            if(strcmp(input('Do you want to change the path points? y/n\n', 's'), 'y'))
                %loadOccupancyMatrix
                load('../mat_files/occupancyMatrix.mat', 'occupancyMatrix');
                MAP_info = load('../mat_files/mapInformation.mat');
                load('../mat_files/safe_matrix.mat', 'safe_matrix');
%                 MAP = figure('Name','MAP','NumberTitle','off');
%                 pbaspect([1 1 1]);
%                 %I = imread('mat_files/MAP_w_roads.png');
%                 MAP.Children.Position = [0 0 1 1];
%                 imshow(I);
%                 MAP.WindowStyle = 'docked';
%                 MAP.Units = 'pixel';
                pathPoints = pickPathPoints(occupancyMatrix);
            else
                disp("What the hell are you tring to change... pls change your mind!")
                return;
            end
        end
    end
    defaultFunction(occupancyMatrix, pathPoints, MAP_info, safe_matrix);
else
    defaultFunction();
end

function [] = defaultFunction(occupancyMatrix, pathPoints, MAP_info, safe_matrix)
    if nargin < 1
        disp('USING DEFAULT CONFIGURATIONS...');
        load('../mat_files/occupancyMatrix.mat', 'occupancyMatrix');
        load('../test_files/pathPoints.mat', 'pathPoints');
        MAP_info = load('../mat_files/mapInformation.mat');
        load('../mat_files/safe_matrix.mat', 'safe_matrix');
    end

    
    [dim_y, dim_x] = size(occupancyMatrix);
    nx = 50;
    ny = 50;
    dx = 1:nx:dim_x;
    dy = 1:ny:dim_y;
    [X,Y] = meshgrid(dx,dy);
    graph = occupancyMatrix(dy,dx)~=0;
    f_aux = figure('WindowStyle', 'docked', 'Units' ,'pixel');
    mesh(X,Y,graph);
    pbaspect([1 1 1]);
    hold on

    [pathPoints_graph,~] = get_closest_point_in_graph(nx,ny,pathPoints,occupancyMatrix);
    plot(pathPoints_graph(:,1), pathPoints_graph(:,2), 'ro')
    view(0,-90);

    xlim([1 dx(end)]);
    ylim([1 dy(end)]);

    Image = getframe(gcf);
    imwrite(Image.cdata, '../mat_files/graph_w_points.png', 'png');
    %close(f_aux)

    % plot new image
    figure('WindowStyle', 'docked');
    I = imread('../mat_files/graph_w_points.png');
    imshow(I)
    
    
    grid.x = X;
    grid.y = Y;
    grid.graph=graph;
    save('../mat_files/grid.mat','-struct','grid');

    

end


function [points,allowed_points] = get_closest_point_in_graph(nx,ny,xy,occupancyMatrix)
    
    points = [];    %   vector of neighbours in the graph
    allowed_points = zeros(size(xy,1),1);     %   real points inside the matrix
    %   float numbers inside the square
    rem_x = rem(xy(:,1),nx);
    rem_y = rem(xy(:,2),ny);
    
    %   The four corners of the closest square
    upper_left_corner = [xy(:,1)-rem_x+1, xy(:,2)-rem_y+1];
    upper_right_corner = [upper_left_corner(:,1)+nx, upper_left_corner(:,2)];
    lower_left_corner = [upper_left_corner(:,1) upper_left_corner(:,2)+ny];
    lower_right_corner = [upper_left_corner(:,1)+nx upper_left_corner(:,2)+ny];
    
    % iterate over the corners
    for idx = 1:size(upper_left_corner,1)
        best_corner = [upper_left_corner(idx,:)
                       upper_right_corner(idx,:)
                       lower_left_corner(idx,:)
                       lower_right_corner(idx,:)];

        dist = vecnorm(xy(idx,:)-best_corner,2,2);    % euclidian distance to the corners: (vec, euclidian norm, dim)
        
        [~,I] = sort(dist); % closet neighbour
        
        % verify if the point is inside the road
        for j = 1:length(I)
            if(occupancyMatrix(best_corner(I(j),2), best_corner(I(j),1)) ~= 0)    % point inside the road
                points = [points;best_corner(I(j),:)];
                allowed_points(idx) = 1;
                break
            end
        end
    end

    if(sum(allowed_points) == 0)
        disp("There any any valid points")
    elseif(sum(allowed_points) == 1)
        disp("There are only one available points")
        points = [];
    end
end

