% TODO link - https://notability.com/n/0goKZZ3B87j8BrKcE3ruIi

% STARTING FUNCTION to create map, new roads and new path points
function create_map
    global occupancy_matrix map_information path_points file_path energy_budget path_orientation map_velocity limit_velocity
    limit_velocity = 10; %Default value (only changed if user wants new speed limit zones)
    map_velocity = 30;
    
    global initialPoint_People orientation_people duration_people time_people thd_col
    
    %     path_img = "../maps/IST_campus.png"; %default map image path
%     file_path = "../maps/IST_campus/";
%     try
%         occupancy_matrix = load(string(file_path + 'occupancy_matrix.mat'), 'occupancy_matrix');
%         path_points = load(string(file_path + 'path_points.mat'), 'path_points');
%         path_orientation = load(string(file_path + 'path_orientation.mat'), 'path_orientation');
%         map_information = load(string(file_path + 'map_information.mat'));
%     catch
%     end
    
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
            [occupancy_matrix, path_points, map_information, path_orientation] = map_generation(path_img, true, file_path);
        else
            % Working with one of the predefined maps
            list = {'IST_campus','car_track'};%, 'car_track_square'};
            [index,~] = listdlg('ListString',list,'SelectionMode','single');
            if(isempty(index)); continue; end
            path_img = string("../maps/" + string(list(index)) + "/" + string(list(index)) + ".png");
            file_path = string("../maps/" + string(list(index)) + "/");
            try
                load(string(file_path + "occupancy_matrix.mat"), 'occupancy_matrix');
                map_information = load(string(file_path + 'map_information.mat'));
                load(string(file_path + "path_points.mat"), 'path_points');
                load(string(file_path + 'path_orientation.mat'), 'path_orientation');
            catch
            end

            %Changing the defined roads?
            if strcmpi(questdlg('Do you want to define new roads? ("No" to use the already defined roads)', 'NEW ROAD DEFINITION', 'Yes', 'No', 'No'), 'Yes')
                [occupancy_matrix, path_points, map_information, path_orientation] = map_generation(path_img, false, file_path);
            else
                % Define new obstacles
                if strcmpi(questdlg('Do you want to create new road marks (like traffic lights, crosswalks and obstacles)? ("No" to use the already defined marks)', 'New Road Marks', 'Yes', 'No', 'No'), 'Yes')
                    occupancy_matrix = redefineRoadMarks(occupancy_matrix, file_path, path_img);
                end
                % Pick new path points?
                if strcmpi(questdlg('Do you want to plan a new path? ("No" to use the already defined path)', 'NEW PATH DEFINITION', 'Yes', 'No', 'No'), 'Yes')
                    %load(file_path + "occupancy_matrix.mat", 'occupancy_matrix');
                    [path_points,path_orientation] = pickPathPoints(occupancy_matrix, file_path);
                end
            end
        end
        
        prompt = {'Type the Energy Budget for the Car Travel [J]:', 'Type the Maximum velocity for this map [km/h]:', 'Type the Threshold to be consider as colisionh [m]:' };
        final_params = string(inputdlg(prompt,'Parameters',[1 50], {'5e6','20','0.1'}));
        final_params = str2double(final_params);
        energy_budget = final_params(1);
        map_velocity = final_params(2);
        thd_col = final_params(3);
        
        %energy_budget = str2double(inputdlg("Type the Energy Budget for the Car Travel",'Energy Budget',[1 40], {'1.8e8'}));
        if energy_budget < 0; energy_budget = 0; end
        
        if((isempty(occupancy_matrix) || isempty(map_information) || isempty(path_points) || isempty(path_orientation)) == true); continue; end
        
        break;
    end
    
    % Now it's time to try to load ist breakups to the map
    try
        occupancy_matrix = ISTBreakups(file_path, path_img, occupancy_matrix);
    catch
        disp("No GPS Breakup zones - They will be randomly generated");
    end
    
%     Now that the user changed everything he wants (if nothing changed the variables are simply loaded in the beginning)
     save(string(file_path + "occupancy_matrix.mat"), 'occupancy_matrix');
     save(string(file_path + "map_information.mat"), '-struct','map_information');
     save(string(file_path + "path_points.mat"), 'path_points');
     save(string(file_path + "path_orientation.mat"), 'path_orientation');
%********************** UTILS FUNCTIONS *************************

    %% Create Map function
    function [occupancy_matrix, pathPoints, MAP_info, path_orientation] = map_generation(path_img, newMapInfo, folder_path)
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
            %Check if image is a real image that we can get meters_from_map
            %with coords or just type a meter_from_map ratio for a non real
            %image
            user_option_img_type = questdlg("Is your image just a simple sketch(No need to input coordinates to get a meters-to-pixel ratio)? Or is it a real map(realistic meters-to-pixel conversion)?  ' ", 'confirmation', 'Sketch', 'Real Map', 'Real Map');
            if(strcmpi(user_option_img_type, 'Real Map'))
                MAP_info = scale_map(folder_path);
            else
                MAP_info.meters_from_MAP =  str2double(inputdlg('So how much meters should a pixel be in the image? (eg. typing 0.17 means 17cm for each image pixel','Meters to Pixel Conversion',[1 40], {'0.1'}));
            end
        else
            MAP_info = load(string(folder_path + "map_information.mat"));
        end

        %% Define the roads by drawing polygons
        [X,Y] = meshgrid(1:size(I, 2),1:size(I, 1));
        occupancy_matrix = zeros(size(I, 1), size(I, 2));
        user_option = "No";
        msgbox("Now it's time to draw roads... So draw one closed polygon where you want a road to be! After that a dialog box will pop up for you to choose if you want to define more roads. Don't mind if the roads are overlapped!","Defining the roads", 'modal');
        roads = {};
        while strcmpi(user_option, "No")
            roadMarkers = drawRoads();
            user_option = questdlg("If you're done drawing roads enter 'Yes' ", 'confirmation', 'Yes', 'No', 'Undo', 'No');
            if strcmpi(user_option, 'Undo')
                 % TODO: Find a way to clear the polygon in image... for now it
                 % simply doesn't save the drawn polygon but it remains plotted
                 user_option = "No";
            else
                occupancy_matrix = occupancy_matrix + inpolygon(X, Y, roadMarkers(:,1)', roadMarkers(:,2)' );
                % Save road polygon to file load it after in MAP
                roads{end + 1} = roadMarkers;
            end
        end
        occupancy_matrix(occupancy_matrix>0) = 1; %Normalizing to 1 because 2 roads may be overlapped

         %% Now defining other specificities in the environment (defined in occupancy matrix)
        user_option = 1;
        cross_walks = {};
        traffic_lights = {};
        stop_signs = {};
        speed_limits = {};
        initialPoint_People = double.empty(2,0);
        orientation_people = [];
        duration_people = [];
        time_people = [];
        while (user_option ~= 6)
            user_option = menu('Choose one of the 5 road marks to add ("Im done!" to exit)','Crosswalk','Traffic Light','Stop Sign', 'Speed Limit Zone', 'Pedestrian Crossing', 'Im done!');
            switch user_option
                case 1
                    crossWalk = drawCrosswalk();
                    cross_walks{end + 1} = crossWalk;
                    occupancy_matrix(logical(inpolygon(X, Y, crossWalk(:,1)', crossWalk(:,2)' ) .* occupancy_matrix)) = 2;
                case 2
                    trafficLights = drawTrafficLight();
                    traffic_lights{end + 1} = trafficLights;
                    occupancy_matrix(logical(inpolygon(X, Y, trafficLights(:,1)', trafficLights(:,2)' ) .* occupancy_matrix)) = 3;
                case 3
                    stopSign = drawStopSign();
                    stop_signs{end + 1} = stopSign;
                    occupancy_matrix(logical(inpolygon(X, Y, stopSign(:,1)', stopSign(:,2)' ) .* occupancy_matrix)) = 4;
                case 4
                    speedLimit = drawSpeedLimit();
                    speed_limits{end + 1} = speedLimit;
                    occupancy_matrix(logical(inpolygon(X, Y, speedLimit(:,1)', speedLimit(:,2)' ) .* occupancy_matrix)) = 7;
                case 5
                    %Pedestrian crossing
                     [initial_p_people, orient_people, dur_people, tm_people] = drawPeopleCrossing;
                     initialPoint_People = [initialPoint_People(1,:) initial_p_people(1); initialPoint_People(2,:) initial_p_people(2)];
                     orientation_people = [orientation_people orient_people];
                     duration_people = [duration_people dur_people];
                     time_people = [time_people tm_people];
                otherwise
            end
        end
        
        close;
        save(string(folder_path + "roads.mat"), 'roads');
        save(string(folder_path + "cross_walks.mat"), 'cross_walks');
        save(string(folder_path + "traffic_lights.mat"), 'traffic_lights');
        save(string(folder_path + "stop_signs.mat"), 'stop_signs');
        save(string(folder_path + "speed_limits.mat"), 'speed_limits');
        save(string(folder_path + "occupancy_matrix.mat"), 'occupancy_matrix');
        
        if(~isempty(speed_limits))
            limit_velocity = str2double(inputdlg("Type the Speed Limit for all the limited zones[km/h]",'Speed Limit',[1 40], {'10'}));
            if(limit_velocity > map_velocity); limit_velocity = map_velocity; end
        end
        %% Picking the start and end points and the intermediate ones
        [pathPoints,path_orientation] = pickPathPoints(occupancy_matrix, folder_path);

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
    function occupancy_matrix = redefineRoadMarks(occupancy_matrix, folder_path, path_img)
        %First clear the previous occupancy_matrix roadSigns (making them =
        %1)
        occupancy_matrix(occupancy_matrix > 1) = 1; 
        %% display Map Image
        %load(string(folder_path+"MAP.mat"),'MAP');
        MAP = figure('Name','MAP','NumberTitle','off');
        pbaspect([1 1 1]);
        I = imread(path_img);
        MAP.Children.Position = [0 0 1 1];
        imshow(I);
        MAP.WindowStyle = 'docked';
        MAP.Units = 'pixel';
        hold on;
        
        %MAP = openfig(string(folder_path+"MAP.fig"));
        load(string(folder_path + "roads.mat"), 'roads');
        for idx=roads
            h = drawpolygon('Color','k','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = [130 130 130] / 255;
            h.FaceAlpha = 0.2;
        end
        
        [X,Y] = meshgrid(1:size(occupancy_matrix, 2),1:size(occupancy_matrix, 1));
        
        user_option = 1;
        cross_walks = {};
        traffic_lights = {};
        stop_signs = {};
        speed_limits = {};
        initialPoint_People = double.empty(2,0);
        orientation_people = [];
        duration_people = [];
        time_people = [];
        while (user_option ~= 6)
            user_option = menu('Choose one of the 5 road marks to add ("Im done!" to exit)','Crosswalk','Traffic Light','Stop Sign', 'Speed Limit Zone', 'Pedestrian Crossing', 'Im done!');
            switch user_option
                case 1
                    crossWalk = drawCrosswalk();
                    cross_walks{end + 1} = crossWalk;
                    occupancy_matrix(logical(inpolygon(X, Y, crossWalk(:,1)', crossWalk(:,2)' ) .* occupancy_matrix)) = 2;
                case 2
                    trafficLights = drawTrafficLight();
                    traffic_lights{end + 1} = trafficLights;
                    occupancy_matrix(logical(inpolygon(X, Y, trafficLights(:,1)', trafficLights(:,2)' ) .* occupancy_matrix)) = 3;
                case 3
                    stopSign = drawStopSign();
                    stop_signs{end + 1} = stopSign;
                    occupancy_matrix(logical(inpolygon(X, Y, stopSign(:,1)', stopSign(:,2)' ) .* occupancy_matrix)) = 4;
                case 4
                    speedLimit = drawSpeedLimit();
                    speed_limits{end + 1} = speedLimit;
                    occupancy_matrix(logical(inpolygon(X, Y, speedLimit(:,1)', speedLimit(:,2)' ) .* occupancy_matrix)) = 7;
                case 5
                    %Pedestrian crossing
                     [initial_p_people, orient_people, dur_people, tm_people] = drawPeopleCrossing;
                     initialPoint_People = [initialPoint_People(1,:) initial_p_people(1); initialPoint_People(2,:) initial_p_people(2)];
                     orientation_people = [orientation_people orient_people];
                     duration_people = [duration_people dur_people];
                     time_people = [time_people tm_people];
                otherwise
            end
        end
        
        if(~isempty(speed_limits))
            limit_velocity = str2double(inputdlg("Type the Speed Limit for all the limited zones[km/h]",'Speed Limit',[1 40], {'10'}));
            if(limit_velocity > map_velocity); limit_velocity = map_velocity; end
        end
        
        savefig(MAP, string(folder_path + "MAP.fig"), 'compact');
        save(string(folder_path + "cross_walks.mat"), 'cross_walks');
        save(string(folder_path + "traffic_lights.mat"), 'traffic_lights');
        save(string(folder_path + "stop_signs.mat"), 'stop_signs');
        save(string(folder_path + "speed_limits.mat"), 'speed_limits');
        close;
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

            prompt = {'Latitude[-90 ; 90]:', 'Longitude[-180 ; 180]:'};
            point1coords = str2double(inputdlg(prompt,'Coordinates - Point 1',[1 50], {'38.736798','-9.139866'}));
            lat1 = point1coords(1);
            if(lat1 < -90); lat1 = -90; end % Latitudes have to be between -90 and 90 [degrees]
            if(lat1 > 90); lat1 = 90; end
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

            prompt = {'Latitude[-90 ; 90]:', 'Longitude[-180 ; 180]:'};
            point2coords = str2double(inputdlg(prompt,'Coordinates - Point 2',[1 50], {'38.736258','-9.137637'}));
            lat2 = point2coords(1);
            if(lat2 < -90); lat1 = -90; end
            if(lat2 > 90); lat1 = 90; end
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

    % Function to pick points of the path and save them
    function [pathPoints, path_orientation] = pickPathPoints(occupancy_matrix, folder_path)
        if nargin < 1
            load(string(folder_path + "occupancy_matrix.mat"), 'occupancy_matrix');
        end
        
        %load(string(file_path+"MAP.mat"),'MAP');
        %MAP = openfig(string(file_path+"MAP.fig"));
        MAP = figure('Name','MAP','NumberTitle','off');
        pbaspect([1 1 1]);
        I = imread(path_img);
        MAP.Children.Position = [0 0 1 1];
        imshow(I);
        MAP.WindowStyle = 'docked';
        MAP.Units = 'pixel';
        hold on;
        %TODO: Load roads and roadmarks to fig
        load(string(folder_path + "roads.mat"), 'roads');
        for idx=roads
            h = drawpolygon('Color','k','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = [130 130 130] / 255;
            h.FaceAlpha = 0.2;
        end
        
        load(string(folder_path + "cross_walks.mat"), 'cross_walks');
        load(string(folder_path + "traffic_lights.mat"), 'traffic_lights');
        load(string(folder_path + "stop_signs.mat"), 'stop_signs');
        load(string(folder_path + "speed_limits.mat"), 'speed_limits');
        for idx=cross_walks
            h = drawpolygon('Color','w','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = 'white';
            h.FaceAlpha = 0.1;
        end
        for idx=traffic_lights
            h = drawpolygon('Color','b','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = 'blue';
            h.FaceAlpha = 0.1;
         end
        for idx=stop_signs
            h = drawpolygon('Color','r','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = 'red';
            h.FaceAlpha = 0.1;
        end
        for idx=speed_limits
            h = drawpolygon('Color','c','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = 'cyan';
            h.FaceAlpha = 0.1;
        end
        
        msgbox("Pick in the map the INITIAL point, the intermediate points and the FINAL point","PICKING PATH POINTS", 'modal');

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
            x = max(min(size(occupancy_matrix,2),h1.Position(1)),1);
            y = max(min(size(occupancy_matrix,1),h1.Position(2)),1);
            if ( occupancy_matrix( round(y), round(x) ) == 0)
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
        end

        prompt = {'Orientation of Initial Point (degrees):', 'Orientation of Ending Point (degrees):'};
        path_orientation = string(inputdlg(prompt,'Definition of Orientations',[1 50], {'0','0'}));
        path_orientation = str2double(path_orientation);
        path_orientation = wrapTo360(path_orientation);
        
        savefig(MAP, string(folder_path + "MAP.fig"), 'compact');
        save(string(folder_path + "path_orientation.mat"), 'path_orientation');
        save(string(folder_path + "path_points.mat"), 'path_points');
        Image = getframe(gcf);
        imwrite(Image.cdata, folder_path + "MAP_w_roads.png", 'png');
        
        close;
    end

    %% Functions to draw specificities of the route (Crossroads, TrafficLights, bumps or holes)

    function stopSign = drawStopSign
        disp("Draw in the map one stop sign by drawing the region");
        h = drawpolygon('Color','r','InteractionsAllowed','none');

        h.Color = 'red';
        h.FaceAlpha = 0.2;
        %h.Label = 'STOP';
        stopSign = h.Position;
    end

    function roadMarkers = drawRoads    
        disp("Draw in the map one road");
        h = drawpolygon('Color','k','InteractionsAllowed','none');

        % Change road color the mark it's done and retrieve pointer positions
        % that mark the road
        h.Color = [130 130 130] / 255;
        h.FaceAlpha = 0.2;
        %h.Label = 'Road';
        roadMarkers = h.Position;
    end

    function crosswalk = drawCrosswalk
        disp("Draw in the map one crosswalk by drawing the region");
        h = drawpolygon('Color','w','InteractionsAllowed','none');

        h.Color = 'white';
        h.FaceAlpha = 0.2;
        %h.Label = 'Cross Walk';
        crosswalk = h.Position;
    end

    function trafficLight = drawTrafficLight
        disp("Draw in the map one traffic light by drawing the region");
        h = drawpolygon('Color','b','InteractionsAllowed','none');

        h.Color = 'blue';
        h.FaceAlpha = 0.2;
        %h.Label = 'Traffic Light';
        trafficLight = h.Position;
    end

    function speedLimit = drawSpeedLimit
        disp("Draw in the map one speed limit zone by drawing the region");
        h = drawpolygon('Color','c','InteractionsAllowed','none');

        h.Color = 'cyan';
        h.FaceAlpha = 0.2;
        %h.Label = 'Speed Limit';
        speedLimit = h.Position;
    end

    function [initialPoint_People, orientation_people, duration_people, time_people] = drawPeopleCrossing
        %Pick a person position, angle of movement, duration of movement
        %and instant of movement
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
        
        hc = drawcircle('Center',h1.Position,'Radius',14,'StripeColor','magenta');
        hc.InteractionsAllowed = 'none';
        initialPoint_People = round(h1.Position);
        
        prompt = {'Type the Angle Orientation of the Pedestrian Movement [degreed]:', 'Type the Movement Duration [s]:', 'Type the start instant of the Movement [s]:'};
        final_params = string(inputdlg(prompt,'Parameters',[1 50], {'180','5','30'}));
        final_params = str2double(final_params);
        orientation_people = final_params(1);
        duration_people = final_params(2);
        time_people = final_params(3);
        
    end

    function occupancy_matrix = ISTBreakups(folder_path, path_img, occupancy_matrix)
        if exist(string(folder_path + "IST_BreakUps.mat"),'file')
            [X,Y] = meshgrid(1:size(occupancy_matrix, 2),1:size(occupancy_matrix, 1));
            %load(string(file_path+"MAP.mat"),'MAP');
            %MAP = openfig(string(file_path+"MAP.fig"));
            MAP = figure('Name','MAP','NumberTitle','off');
            pbaspect([1 1 1]);
            I = imread(path_img);
            MAP.Children.Position = [0 0 1 1];
            imshow(I);
            MAP.WindowStyle = 'docked';
            MAP.Units = 'pixel';
            hold on;
            load(string(folder_path + "roads.mat"), 'roads');
            for idx=roads
                h = drawpolygon('Color','k','InteractionsAllowed','none', 'Position', cell2mat(idx));
                h.Color = [130 130 130] / 255;
                h.FaceAlpha = 0.2;
            end
            
            load(string(folder_path + "cross_walks.mat"), 'cross_walks');
            load(string(folder_path + "traffic_lights.mat"), 'traffic_lights');
            load(string(folder_path + "stop_signs.mat"), 'stop_signs');
            load(string(folder_path + "speed_limits.mat"), 'speed_limits');
            for idx=cross_walks
                h = drawpolygon('Color','w','InteractionsAllowed','none', 'Position', cell2mat(idx));
                h.Color = 'white';
                h.FaceAlpha = 0.1;
            end
            for idx=traffic_lights
                h = drawpolygon('Color','b','InteractionsAllowed','none', 'Position', cell2mat(idx));
                h.Color = 'blue';
                h.FaceAlpha = 0.1;
            end
            for idx=stop_signs
                h = drawpolygon('Color','r','InteractionsAllowed','none', 'Position', cell2mat(idx));
                h.Color = 'red';
                h.FaceAlpha = 0.1;
            end
            for idx=speed_limits
                h = drawpolygon('Color','c','InteractionsAllowed','none', 'Position', cell2mat(idx));
                h.Color = 'cyan';
                h.FaceAlpha = 0.1;
            end
            %TODO: Load roads and roadmarks to fig
            load(string(folder_path + "IST_BreakUps.mat"), 'ISTBreakupPoints');
            for idx=ISTBreakupPoints
                h = drawpolygon('Color','y','InteractionsAllowed','none', 'Position', cell2mat(idx));
                h.Color = [255 204 102] / 255;
                h.FaceAlpha = 0.2;
                gps_breakup = cell2mat(idx);
                occupancy_matrix(logical(inpolygon(X, Y, gps_breakup(:,1)', gps_breakup(:,2)' ) .* occupancy_matrix)) = 6;
            end
            savefig(MAP, string(folder_path + "MAP.fig"), 'compact');
        end
    end

    
end
