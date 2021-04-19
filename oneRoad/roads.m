clc;
clear all;

map = imread('map.PNG');
figure;
imshow(map)

roadPos = drawOneRoad()

%% Drawing the road
function roadMarkers = drawOneRoad()
    road_width = -1;
    number_of_ways = -1;

    disp("Draw in the map one road (Start by picking 1 starting point and the ending point of the road: )");
    while 1
        h = drawpolyline('Color','blue');
        
        while 1
            road_width = str2double(input("\nEnter the road width in [m]: ", 's'));
            if(road_width > 0 && not(isnan(road_width)))
                break;
            end
        end
        
         while 1
            number_of_ways = str2double(input("\nEnter the number of ways the road has [1 or 2 ways]: ", 's'));
            if(number_of_ways > 0 && number_of_ways < 3  && not(isnan(number_of_ways)))
                break;
            end
        end

        %Because Matlab doesn't have a do...while I have to do this
        if ( strcmp(input("\nIf you're done drawing this road enter 'Y', if not press 'N': ", 's'), 'Y') == 1 )
            break;
        end
    end

    % Change road color the mark it's done and retrieve pointer positions that
    % mark that roda
    h.Color = 'green';
    roadMarkers = h.Position;
end