clc;
close all;
clear variables;
load('../../occupancyMatrix.mat', 'occupancyMatrix');
load('../../run_points.mat', 'run_points');

 [change_points, cluster] = pathSegmentation(run_points);
 save('../mat_files/segmentation.mat', 'change_points', 'cluster');

function  [change_points, cluster] = pathSegmentation(run_points)
    a = diff(run_points);
    a = [a(1, :); a];
    slopes = atan2(a(:, 2),a(:,1));

    diff_slopes = diff(slopes);
    diff_slopes(diff_slopes ~=0);
    diff_slopes = logical(diff_slopes);
    diff_slopes(end+1) = false;

    change_points = run_points;
    change_points(~diff_slopes,:) = [];

    aware_distance = 10; %meters
    threshold = round(aware_distance/0.1764);
    cluster = zeros(length(change_points),  1);
    counter = 0;
    for i = 1:length(change_points)-1
        %new clusteri
        dist = norm(change_points(i, :) - change_points(i+1,:));
        if  dist > threshold   
            counter = counter +1;
        end
        cluster(i+1) = counter;
    end
    figure();
    plot(run_points(:, 1), run_points(:, 2), 'b');
    hold on
    gscatter(change_points(:, 1), change_points(:, 2), cluster);
    axis equal
end








