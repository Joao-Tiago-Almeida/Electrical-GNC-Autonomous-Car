clear all;
close all;
clc;
%%
global debug_mode path_points path_orientation max_velocity

max_velocity = 30; %Km/h

debug_mode = true;
create_map;

[sampled_path, checkpoints] = path_planning(path_points, path_orientation);