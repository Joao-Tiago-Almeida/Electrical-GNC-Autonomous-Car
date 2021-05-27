% TODO link - https://notability.com/n/0goKZZ3B87j8BrKcE3ruIi

% STARTING FUNCTION to create map, new roads and new path points
function create_map()
    global occupancy_matrix map_information path_points file_path
    path_img = "../maps/IST_campus.png"; %default map image path
    file_path = "../maps/IST_campus/";
    try
        occupancy_matrix = struct2array(load(string(file_path + 'occupancy_matrix.mat', 'occupancy_matrix')));
        path_points = struct2array(load(string(file_path + 'path_points.mat', 'path_points')));
        map_information = load(string(file_path + 'map_information.mat'));
    catch
    end
    
    %Changing the Map Image?
    while(true)
        if strcmpi(questdlg('Do you want to change the used Map image? ("No" to choose one of the default images)', 'Welcome to the Guidance GUI', 'Yes', 'No', 'No'), 'Yes')
            prompt = {'Input new map path: '};
            path_img = "";
            while(strlength(path_img)<1)    % avoid click in OK
                path_img = string(inputdlg(prompt,'MAP PATH',[1 60],{'','hsv'}));
            end
            if(isempty(path_img)); continue; end
            [pathStr, ~, ~] = fileparts(path_img);
            file_path = string(pathStr + "/");
            [occupancy_matrix, path_points, map_information] = map_generation(path_img, true, file_path);
        else
            % Working with one of the predefined maps
            list = {'IST_campus','car_track'};%, 'car_track_square'};
            [index,~] = listdlg('ListString',list,'SelectionMode','single');
            if(isempty(index)); continue; end
            path_img = string("../maps/" + string(list(index)) + "/" + string(list(index)) + ".png");
            file_path = string("../maps/" + string(list(index)) + "/");
            try
                occupancy_matrix = struct2array(load(string(file_path + "occupancy_matrix.mat"), 'occupancy_matrix'));
                path_points = struct2array(load(string(file_path + "path_points.mat"), 'path_points'));
                map_information = load(string(file_path + 'map_information.mat'));
            catch
            end

            %Changing the defined roads?
            if strcmpi(questdlg('Do you want to define new roads? ("No" to use the already defined roads)', 'NEW ROAD DEFINITION', 'Yes', 'No', 'No'), 'Yes')
                [occupancy_matrix, path_points, map_information] = map_generation(path_img, false, file_path);
            else
                % Pick new path points?
                if strcmpi(questdlg('Do you want to plan a new path? ("No" to use the already defined path)', 'NEW PATH DEFINITION', 'Yes', 'No', 'No'), 'Yes')
                    load(file_path + "occupancy_matrix.mat", 'occupancy_matrix');
                    path_points = pickPathPoints(occupancy_matrix, file_path);
                    %Define new obstacles
                    if strcmpi(questdlg('Do you want to create new road marks (like traffic lights, crosswalks and obstacles)? ("No" to use the already defined marks)', 'New Road Marks', 'Yes', 'No', 'No'), 'Yes')
                        occupancy_matrix = redefineRoadMarks(occupancy_matrix, path_img);
                    end
                else
                    %Define new obstacles
                    if strcmpi(questdlg('Do you want to create new road marks (like traffic lights, crosswalks and obstacles)? ("No" to use the already defined marks)', 'New Road Marks', 'Yes', 'No', 'No'), 'Yes')
                        occupancy_matrix = redefineRoadMarks(occupancy_matrix, path_img);
                    end
                end
            end
        end
        break;
    end
    % TODO só entrar aqui quando o file_path não for empty
    
    %Now that the user changed everything he wants (if nothing changed the variables are simply loaded in the beginning)
    save(string(file_path + "occupancy_matrix.mat"), 'occupancy_matrix');
    save(string(file_path + "map_information.mat"), '-struct','map_information');
    save(string(file_path + "path_points.mat"), 'path_points');


%********************** UTILS FUNCTIONS *************************

    %% Create Map function
    function [occupancy_matrix, pathPoints, MAP_info] = map_generation(path_img, newMapInfo, folder_path)
        %% display Map Image
        MAP = figure('Name','MAP','NumberTitle','off');
        pbaspect([1 1 1]);
        I = imread(path_img);
        MAP.Children.Position = [0 0 1 1];
        imshow(I);
        MAP.WindowStyle = 'docked';
        MAP.Units = 'pixel';
        hold on;

        %% set MAP scale
        if newMapInfo == true
            MAP_info = scale_map(folder_path);
        else
            MAP_info = load(string(folder_path + "map_information.mat"));
        end

        %% Define the roads by drawing polygons
        [X,Y] = meshgrid(1:size(I, 2),1:size(I, 1));
        occupancy_matrix = zeros(size(I, 1), size(I, 2));
        user_option = "No";
        msgbox("Now it's time to draw roads... So draw one closed polygon where you want a road to be! After that a dialog box will pop up for you to choose if you want to define more roads. Don't mind if the roads are overlapped!","Defining the roads", 'modal');
        while strcmpi(user_option, "No")
            roadMarkers = drawRoads();
            user_option = questdlg("If you're done drawing roads enter 'Yes' ", 'confirmation', 'Yes', 'No', 'Undo', 'No');
            if strcmpi(user_option, 'Undo')
                 % TODO: Find a way to clear the polygon in image... for now it
                 % simply doesn't save the drawn polygon but it remains plotted
                 user_option = "No";
            else
                occupancy_matrix = occupancy_matrix + inpolygon(X, Y, roadMarkers(:,1)', roadMarkers(:,2)' );
            end
        end

        occupancy_matrix(occupancy_matrix>0) = 1; %Normalizing to 1 because 2 roads may be overlapped

         %% Now defining other specificities in the environment (defined in occupancy matrix)
        user_option = 1;
        while (user_option ~= 4)
            user_option = menu('Choose one of the 3 road marks to add ("Im done!" to exit)','Crosswalk','Traffic Light','Obstacle', 'Im done!');
            switch user_option
                case 1
                    crossWalk = drawCrosswalk();
                    occupancy_matrix(logical(inpolygon(X, Y, crossWalk(:,1)', crossWalk(:,2)' ) .* occupancy_matrix)) = 2;
                case 2
                    trafficLights = drawTrafficLight();
                    occupancy_matrix(logical(inpolygon(X, Y, trafficLights(:,1)', trafficLights(:,2)' ) .* occupancy_matrix)) = 3;
                case 3
                    stopSign = drawStopSign();
                    occupancy_matrix(logical(inpolygon(X, Y, stopSign(:,1)', stopSign(:,2)' ) .* occupancy_matrix)) = 4;
                otherwise
            end
        end
       
        
        save(string(folder_path + "MAP.mat"), 'MAP');
        save(string(folder_path + "occupancy_matrix.mat"), 'occupancy_matrix');
        Image = getframe(gcf);
        pause(0.2)
        imwrite(Image.cdata, string(folder_path + "MAP_w_roads.png"));

        %% Picking the start and end points and the intermediate ones
        pathPoints = pickPathPoints(occupancy_matrix, folder_path);

        %% draw speacial regions on the map

        figure('WindowStyle', 'docked');
        mesh(occupancy_matrix)
        colorTheme = [ 0 0 0
            128 128 128
            255 255 255
            0 255 0
            255 0 0
            ]/255;
        colormap(colorTheme);
    end

   %Redefine Road Marks (Traffic lights, crosswalks and obstacles)
    function occupancy_matrix = redefineRoadMarks(occupancy_matrix, path_img)
        %First clear the previous occupancy_matrix roadSigns (making them =
        %1)
%         occupancy_matrix = struct2array(occupancy_matrix);
        occupancy_matrix(occupancy_matrix > 1) = 1; 
        %% display Map Image
        MAP = figure('Name','MAP','NumberTitle','off');
        pbaspect([1 1 1]);
        I = imread(path_img);
        MAP.Children.Position = [0 0 1 1];
        imshow(I);
        MAP.WindowStyle = 'docked';
        MAP.Units = 'pixel';
        hold on;
        [X,Y] = meshgrid(1:size(I, 2),1:size(I, 1));
        
        user_option = 1;
        while (user_option ~= 4)
            user_option = menu('Choose one of the 3 road marks to add ("Im done!" to exit)','Crosswalk','Traffic Light','Obstacle', 'Im done!');
            switch user_option
                case 1
                    crossWalk = drawCrosswalk();
                    occupancy_matrix(logical(inpolygon(X, Y, crossWalk(:,1)', crossWalk(:,2)' ) .* occupancy_matrix)) = 2;
                case 2
                    trafficLights = drawTrafficLight();
                    occupancy_matrix(logical(inpolygon(X, Y, trafficLights(:,1)', trafficLights(:,2)' ) .* occupancy_matrix)) = 3;
                case 3
                    stopSign = drawStopSign();
                    occupancy_matrix(logical(inpolygon(X, Y, stopSign(:,1)', stopSign(:,2)' ) .* occupancy_matrix)) = 4;
                otherwise
            end
        end
        
    end

    % Scaling the map to get map informations
    function MAP_info = scale_map(folder_path)
        %% get the coordinates
        % Right green lamp in the star exit of the roundabout
        %lat1 =  38.736798;
        %lon1 = -9.139866;

        % Center of the futsal camp
        %lat2 =  38.736258;
        %lon2 = -9.137637;

        user_option = "No";
        msgbox("Draw the FIRST location point, and type the coordinates","Defining the references", 'modal');
        while(strcmpi(user_option, "No"))
            try
                h1 = drawcrosshair('LineWidth',1, 'Color','k');
            catch
                h1 = drawpoint('LineWidth',1, 'Color','k');
            end

            prompt = {'Latitude:', 'Longitude:'};
            point1coords = str2double(inputdlg(prompt,'Coordinates - Point 1',[1 50], {'38.736798','-9.139866'}));
            lat1 = point1coords(1);
            lon1 = point1coords(2);

            user_option = questdlg("If you're satisfied with the picked point enter 'Yes' to go pick the SECOND point ", 'confirmation', 'Yes', 'No', 'Undo', 'Yes');
            try
                h1.Visible = 'off';
            catch
                h1.Visibility = 'off';
            end
        end

        x1 = round(h1.Position);

        user_option = "No";
        %msgbox("Draw the SECOND location point, and type the coordinates","Defining the references", 'modal');
        while(strcmpi(user_option, "No"))
            try
                h2 = drawcrosshair('LineWidth',1, 'Color','k');
            catch
                h2 = drawpoint('LineWidth',1, 'Color','k');
            end

            prompt = {'Latitude:', 'Longitude:'};
            point2coords = str2double(inputdlg(prompt,'Coordinates - Point 2',[1 50], {'38.736258','-9.137637'}));
            lat2 = point2coords(1);
            lon2 = point2coords(2);

            user_option = questdlg("If you're satisfied with the picked point enter 'Yes' ", 'confirmation', 'Yes', 'No', 'Undo', 'Yes');
            try
                h2.Visible = 'off';
            catch
                h2.Visibility = 'off';
            end
        end

        x2 = round(h2.Position);

        %% get the distance

        % compute distances between to geolocated points
        radius=6371e3;
        lat1=lat1*pi/180;
        lat2=lat2*pi/180;
        lon1=lon1*pi/180;
        lon2=lon2*pi/180;
        deltaLat=lat2-lat1;
        deltaLon=lon2-lon1;
        a=sin((deltaLat)/2)^2 + cos(lat1)*cos(lat2) * sin(deltaLon/2)^2;
        c=2*atan2(sqrt(a),sqrt(1-a));
        dist_geo=radius*c;    %Haversine distance

        dist_MAP = vecnorm(x1-x2);

        % return values - approx planar zone
        MAP_info.meters_from_MAP = dist_geo/dist_MAP;    % relation between 
        MAP_info.fget_Lat_from_MAP = @(x) (deltaLat/(x2(2)- x1(2))*x + 0.5*(lat1+lat2-(x2(2)+x1(2))*deltaLat/(x2(2)- x1(2))) )*180/pi;   % computes the latitute from the y coordinate
        MAP_info.fget_Lon_from_MAP = @(x) (deltaLon/(x2(1)- x1(1))*x + 0.5*(lon1+lon2-(x2(1)+x1(1))*deltaLon/(x2(1)- x1(1))) )*180/pi;   % computes the longitude from the x coordinate

        save(string(folder_path + "map_information.mat"),'-struct','MAP_info');
    end

    % Because given points are not exactly over the grid lines so we need to aproximate them to the grid 
    function [points,allowed_points] = get_closest_point_in_grid(nx,ny,xy,occupancy_matrix, folder_path)
        points = [];    %   vector of neighbours in the grid
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
                if(occupancy_matrix(best_corner(I(j),2), best_corner(I(j),1)) ~= 0)    % point inside the road
                    points = [points;best_corner(I(j),:)];
                    allowed_points(idx) = 1;
                    break
                end
            end
        end

        if(sum(allowed_points) == 0)
            msgbox("Unfortunately none of the path points are valid!!!","Error", 'error', 'modal');
        elseif(sum(allowed_points) == 1)
            msgbox("Unfortunately there is only ONE valid point... No path can be built from just 1 point!!!","Error", 'error', 'modal');
            points = [];
        end
        save(string(folder_path + "pathPoints_grid.mat"), 'points');
    end

    % Function to pick points of the path and save them
    function pathPoints = pickPathPoints(occupancy_matrix, folder_path)
        if nargin < 1
            load(string(folder_path + "occupancy_matrix.mat"), 'occupancy_matrix');
        end
        msgbox("Pick in the map the INITIAL point, the intermediate points and the FINAL point","PICKING PATH POINTS", 'modal');

        imshow(occupancy_matrix);

        pathPoints = [];
        user_option = "No";
        while(strcmpi(user_option, "No") || size(pathPoints, 1) < 2)
            try
                h1 = drawcrosshair('LineWidth',1, 'Color','k');
            catch
                h1 = drawpoint('LineWidth',1, 'Color','k');
            end

            try
                h1.Visible = 'off';
            catch
                h1.Visibility = 'off';
            end
            if ( occupancy_matrix( round(h1.Position(2)), round(h1.Position(1)) ) == 0)
                msgbox("Please pick a point inside a road!!!","Error Picking Point", 'error', 'modal');
                continue;
            end

            hc = drawcircle('Center',h1.Position,'Radius',14,'StripeColor','cyan');
            hc.InteractionsAllowed = 'none';
            pathPoints = [pathPoints; round(h1.Position)];

            user_option = questdlg("If you're done picking points enter 'Yes' ", 'confirmation', 'Yes', 'No', 'No');
            if(strcmpi(user_option, "Yes") && (size(pathPoints, 1) < 2) )
                msgbox("Insert at least two points!","Error Picking Point", 'error', 'modal');
                user_option = "No";
                continue;
            end

            h.Color = 'magenta';
            h.FaceAlpha = 0.8;
        end

        save(string(folder_path + "path_points.mat"), 'pathPoints');
        Image = getframe(gcf);
        imwrite(Image.cdata, folder_path + "MAP_w_roads.png", 'png');
    end

    %% Functions to draw specificities of the route (Crossroads, TrafficLights, bumps or holes)

    function stopSign = drawStopSign
        disp("Draw in the map one stop sign by drawing the region");
        h = drawpolygon('Color','r','InteractionsAllowed','none');

        h.Color = 'red';
        h.FaceAlpha = 0.1;
        h.Label = 'STOP';
        stopSign = h.Position;
    end

    function roadMarkers = drawRoads    
        disp("Draw in the map one road");
        h = drawpolygon('Color','k','InteractionsAllowed','none');

        % Change road color the mark it's done and retrieve pointer positions
        % that mark the road
        h.Color = [130 130 130] / 255;
        h.FaceAlpha = 0.1;
        h.Label = 'Road';
        roadMarkers = h.Position;
    end

    function crosswalk = drawCrosswalk
        disp("Draw in the map one crosswalk by drawing the region");
        h = drawpolygon('Color','w','InteractionsAllowed','none');

        h.Color = 'white';
        h.FaceAlpha = 0.1;
        h.Label = 'Cross Walk';
        crosswalk = h.Position;
    end

    function trafficLight = drawTrafficLight
        disp("Draw in the map one traffic light by drawing the region");
        h = drawpolygon('Color','g,','InteractionsAllowed','none');

        h.Color = 'green';
        h.FaceAlpha = 0.1;
        h.Label = 'Traffic Light';
        trafficLight = h.Position;
    end

    
end
