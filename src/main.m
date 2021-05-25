clear all;
close all;
clc;
%%
global debug_mode path_points

debug_mode = true;
create_map

[smoothed_path, checkpoints] = path_planning(path_points);