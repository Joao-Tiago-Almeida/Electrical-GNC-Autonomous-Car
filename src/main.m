clear all;
close all;
clc;
%%
global debug_mode path_points path_orientation

debug_mode = true;
create_map
path_orientation=[0;0];

[sampled_path, checkpoints] = path_planning(path_points, path_orientation);
