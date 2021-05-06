function [MAP_info] = scale_map()
clc
%% get the coordinates

% Right green lamp in the star exit of the roundabout
lat1 =  38.736798;
lon1 = -9.139866;

% Center of the futsal camp
lat2 =  38.736258;
lon2 = -9.137637;

point_not_valid = true;
disp("Draw the first location point, and type the coordinates.");
while(point_not_valid)
    try
        h1 = drawcrosshair('LineWidth',1, 'Color','k');
    catch
        h1 = drawpoint('LineWidth',1, 'Color','k');
    end
    
%     lat1 = input("Latitude: ");
%     lon1 = input("Longitude: ");
    
    valid = input("If you want to draw it again, type 'draw'. If not press any other key. ", 's');
    if(strcmp(valid, "draw")==0)
        point_not_valid = false;
    end
    
    try
        h1.Visible = 'off';
    catch
        h1.Visibility = 'off';
    end
end

x1 = round(h1.Position);
%plot(x1(1), x1(2), '+g', 'MarkerSize', 30, 'LineWidth', 3);

point_not_valid = true;
disp("Draw the second location point, and type the coordinates.");
while(point_not_valid)
    try
        h2 = drawcrosshair('LineWidth',1, 'Color','k');
    catch
        h2 = drawpoint('LineWidth',1, 'Color','k');
    end
% 
%     lat2 = input("Latitude: ");
%     lon2 = input("Longitude: ");
    
    valid = input("If you want to draw it again, type 'draw'. If not press any other key. ", 's');
    if(strcmp(valid, "draw")==0)
        point_not_valid = false;
    end
    
    try
        h2.Visible = 'off';
    catch
        h2.Visibility = 'off';
    end
end

x2 = round(h2.Position);
%plot(x2(1), x2(2), 'xg', 'MarkerSize', 30, 'LineWidth', 3);

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

%save('../mat_files/mapInformation.mat','-struct','MAP_info');
save('../test_files/mapInformation.mat','-struct','MAP_info');

end

