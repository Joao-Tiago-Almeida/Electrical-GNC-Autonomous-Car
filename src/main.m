clear;
close all;
clc;
%%
disp('WELCOME TO THE GRAPHICAL GUIDANCE USER INTERFACE')
ans = input('Do you want to change the default configurations? y/n\n', 's');
if(strcmp(ans, 'y'))
    if(strcmp(input('Do you want to change the map? y/n\n', 's'), 'y'))
        path_img = input('Input new map path', 's');
        [occupancyMatrix, pathPoints, MAP_info] = create_map(path_img);
        
    else 
        [occupancyMatrix, pathPoints, MAP_info] = create_map();
        
    end
    defaultFunction(occupancyMatrix, pathPoints, MAP_info);
else
    defaultFunction();
end

function [] = defaultFunction(occupancyMatrix, pathPoints, MAP_info)
    if nargin < 1
        disp('USING DEFAULT CONFIGURATIONS...')
        occupancyMatrix = load('../binaryMatrix.mat');
        occupancyMatrix = occupancyMatrix.binaryMatrix;
        load('../pathPoints.mat');
        load('../mapInformation.mat');
    end
    
    initialPoint = [ceil(pathPoints(1).Position(2)), ceil(pathPoints(1).Position(1))]
    
    % FUNCTION THAT TRACKS THE BOUNDARIES
    [B,L,n, A] = bwboundaries(occupancyMatrix);
    imshow(occupancyMatrix); hold on;
    colors=['b' 'g' 'r' 'c' 'm' 'y'];
    for k=1:length(B)
      boundary = B{k};
      cidx = mod(k,length(colors))+1;
      plot(boundary(:,2), boundary(:,1),...
           colors(cidx),'LineWidth',2);

      %randomize text position for better visibility
      rndRow = ceil(length(boundary)/(mod(rand*k,7)+1));
      col = boundary(rndRow,2); row = boundary(rndRow,1);
      h = text(col+1, row-1, num2str(L(row,col)));
      set(h,'Color',colors(cidx),'FontSize',14,'FontWeight','bold');
    end
    
    %search the region of the starting point
    
    for i= 1:length(B)
        pol = polyshape(B{i}(:, 1), B{i}(:, 2));
        plot(pol.Vertices(:, 2), pol.Vertices(:, 1));
        
        if(inpolygon(initialPoint(2), initialPoint(1), pol.Vertices(:, 2), pol.Vertices(:, 1)))
            %turns sparsed A to a logical matrix
            not_sparsed = full(A);
            %finds correlation/ holes of the polygon
            holes_index = find(not_sparsed(:, i));
            break;
        end
    end
    for i=1:length(holes_index)
        pol = addboundary(pol, B{holes_index(i)});
    end
    figure();
    plot(pol.Vertices(:, 2), pol.Vertices(:, 1));
    [connectivityMatrix, nodesPosition] = create_graph(pol);
end








