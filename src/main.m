clear;
close all;
clc;
%%
disp('WELCOME TO THE GRAPHICAL GUIDANCE USER INTERFACE')
using_default_configurations = input('Do you want to change the default configurations? y/n\n', 's');
if(strcmp(using_default_configurations, 'y'))
    if(strcmp(input('Do you want to change the map? y/n\n', 's'), 'y'))
        path_img = input('Input new map path', 's');
        [occupancyMatrix, pathPoints, MAP_info, safe_matrix] = create_map(path_img);
        
    else 
        [occupancyMatrix, pathPoints, MAP_info, safe_matrix] = create_map();
        
    end
    defaultFunction(occupancyMatrix, pathPoints, MAP_info);
else
    defaultFunction();
end

function [] = defaultFunction(occupancyMatrix, pathPoints, MAP_info, safe_matrix)
    if nargin < 1
        disp('USING DEFAULT CONFIGURATIONS...');
        load('../mat_files/occupancyMatrix.mat', 'occupancyMatrix');
        load('../mat_files/pathPoints.mat', 'pathPoints');
        MAP_info = load('../mat_files/mapInformation.mat');
        load('../mat_files/safe_matrix.mat', 'safe_matrix');
    end
    
    [dim_y, dim_x] = size(occupancyMatrix);
    nx = 50;
    ny = 50;
    dx = 1:nx:dim_x;
    dy = 1:ny:dim_y;
    [X,Y] = meshgrid(dx,dy);
    graph = occupancyMatrix(dx,dy)~=0;
    mesh(X,Y,graph);
    pbaspect([1 1 1]);
    hold on
    plot(pathPoints(:,1), pathPoints(:,2), 'ro')
    view(0,-90);
    
    grid.x = X;
    grid.y = Y;
    grid.graph=graph;
    save('../mat_files/grid.mat','-struct','grid');

    
%     initialPoint = [ceil(Position(1)), Position(2))];
%    
%     % FUNCTION THAT TRACKS THE BOUNDARIES
%     [B,L,n, A] = bwboundaries(occupancyMatrix);
%     imshow(occupancyMatrix); hold on;
%     colors=['b' 'g' 'r' 'c' 'm' 'y'];
%     for k=1:length(B)
%       boundary = B{k};
%       cidx = mod(k,length(colors))+1;
%       plot(boundary(:,2), boundary(:,1),...
%            colors(cidx),'LineWidth',2);
% 
%       %randomize text position for better visibility
%       rndRow = ceil(length(boundary)/(mod(rand*k,7)+1));
%       col = boundary(rndRow,2); row = boundary(rndRow,1);
%       h = text(col+1, row-1, num2str(L(row,col)));
%       set(h,'Color',colors(cidx),'FontSize',14,'FontWeight','bold');
%     end
%     
%     %search the region of the starting point
%     
%     for i= 1:length(B)
%         pol = polyshape(B{i}(1:end-1, 1), B{i}(1:end-1, 2));
%         plot(pol.Vertices(:, 2), pol.Vertices(:, 1));
%         
%         if(inpolygon(initialPoint(2), initialPoint(1), pol.Vertices(:, 2), pol.Vertices(:, 1)))
%             %turns sparsed A to a logical matrix
%             not_sparsed = full(A);
%             %finds correlation/ holes of the polygon
%             holes_index = find(not_sparsed(:, i));
%             break;
%         end
%     end
%     for i=1:length(holes_index)
%         pol = addboundary(pol, B{holes_index(i)});
%     end
%     figure();
%     plot(pol.Vertices(:, 2), pol.Vertices(:, 1));
%     %[connectivityMatrix, nodesPosition] = create_graph(pol);
%     tic
%     create_graph(pol, logical(occupancyMatrix));
%     toc
end







