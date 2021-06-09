clear all;
close all;
clc;
%%
global debug_mode path_points path_orientation map_information

debug_mode = true;
create_map;

[sampled_path, checkpoints] = path_planning(path_points, path_orientation,"velocity");




%% Web map
% 
% if(length(fieldnames(map_information))==3)
%     webmap
%     lat = map_information.fget_Lat_from_MAP(sampled_path(:,2));
%     lon = map_information.fget_Lon_from_MAP(sampled_path(:,1));
%     wmline(lat,lon)
%     wmmarker(map_information.fget_Lat_from_MAP(checkpoints(:,2)),map_information.fget_Lon_from_MAP(checkpoints(:,1)))
% end

