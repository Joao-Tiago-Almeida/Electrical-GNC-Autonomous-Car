clc;
clear all;

map = imread('map.PNG');
figure;
imshow(map)

height = size(map, 1);
width = size(map, 2);

roadPos = drawOneRoad();

m_size = size(map);

%% viegas isto é muito mais rapido!
tic
[X,Y] = meshgrid(1:width,1:height);
binaryMatrix = inpolygon(X, Y, roadPos(:,1)', roadPos(:,2)' );
figure()
imshow(binaryMatrix)
toc

save('../src/binaryMatrix.mat', 'binaryMatrix');
%%
tic
%Draw the polygon in map image and generate binary matrix with one's in
%polygon
for i=1:size(map, 1)
    for j=1:size(map, 2)
            if inpolygon(i, j, roadPos(:,1)', roadPos(:,2)' ) == 1
                binaryMatrix(i, j) = 1;
                map( linspace(j, j, 5), linspace(i, i, 5), 1) = 100;
                map( linspace(j, j, 5), linspace(i, i, 5), 2) = 100;
                map( linspace(j, j, 5), linspace(i, i, 5), 3) = 100;
            end
    end
end

figure;
imshow(map)
toc
disp("");

%% Drawing the road
function roadMarkers = drawOneRoad()
    disp("Draw in the map one road (Start by picking 1 starting point and the ending point of the road: )");
    h = drawpolygon('Color','blue');
        
%         if ( strcmp(input("\nIf you're done drawing this road enter 'Y', if not press 'N': ", 's'), 'Y') == 1 )
%             break;
%         end

    % Change road color the mark it's done and retrieve pointer positions that
    % mark that road
    h.Color = 'green';
    roadMarkers = h.Position;
end
