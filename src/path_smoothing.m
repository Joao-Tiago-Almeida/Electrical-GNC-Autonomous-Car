
clc;

close all;
clear variables;
load ../mat_files/run_points.mat;
load ../mat_files/occupancyMatrix.mat
checkpoints = [373 209;480 1010; 446 1337; 840 1195];


run_points = insert_checkpoints_in_runPoints(run_points, checkpoints);
[change_points, cluster, threshold, cluster_boundaries] = pathSegmentation(run_points, checkpoints);

smoothed_path = spline_clusters(cluster_boundaries, change_points, cluster, checkpoints);
smoothed_path = [run_points(1,:); smoothed_path'; run_points(end,:)];
save('../mat_files/smoothed_path.mat', 'smoothed_path');

figure;
hold all;
plot(checkpoints(:,1), checkpoints(:,2), 'g*')
plot(run_points(:,1), run_points(:,2), 'r');
plot(smoothed_path(:,1), smoothed_path(:,2), 'b');
legend('Dijsktra Path', 'Smoothed Path', ...
    'Location', 'Best');
title("Our way");
axis equal
hold off;

function run_points = insert_checkpoints_in_runPoints(run_points, checkpoints)
    for i_check = 2:length(checkpoints)-1
        norm_points = vecnorm( run_points - checkpoints(i_check,:), 2, 2 );
        [~, min_index] = min(norm_points);
        run_points(min_index,:) = checkpoints(i_check,:);
    end
end

function M = spline_clusters(cluster_boundaries, change_points, cluster, checkpoints)
    degree = 3;
    number_of_clusters = cluster(end)+1;
    M = [];
    for i = 1:number_of_clusters
        all_cluster_points = [(cluster_boundaries(i,:)); (change_points( cluster(:)==i-1,: )); (cluster_boundaries(i+1,:))];
        %Creation of the knot vector
        x = linspace(0, 1, length(all_cluster_points(:,1)) + degree);
        y = gaussmf(x, [0.5 0.5]);
        t = cumsum(y)/max(cumsum(y));
        
        w = ones(1, length(all_cluster_points));
        for chk_point = 1:length(checkpoints)
            norm_points = ~logical (vecnorm( checkpoints(chk_point,:) - all_cluster_points, 2, 2 ));
            w(norm_points) = 5;
        end
        
        [M_0,x] = bspline_wdeboor(degree,t,all_cluster_points', w, 100);
        M = [M, M_0];
    end
end

function  [change_points, cluster, threshold, cluster_boundaries] = pathSegmentation(run_points, checkpoints)
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
        %new cluster
        dist = norm(change_points(i, :) - change_points(i+1,:));
        if  dist > threshold   
            counter = counter +1;
        end
        cluster(i+1) = counter;        
    end
    
    cluster_division = diff(cluster); %To check the changes in clusters
    cluster_boundaries = checkpoints(1,:); %Starting with the inital path point
    for i = 1:length(cluster_division)
        if cluster_division(i)==0
            continue;
        end
            %If is a new cluster
        cluster_boundaries = [cluster_boundaries; ( (change_points(i,:) + change_points(i+1,:)) / 2 ) ];
    end
    cluster_boundaries = [cluster_boundaries; ( (change_points(i,:) + change_points(i+1,:)) / 2 ) ];
    cluster_boundaries(end,:) =  checkpoints(end,:); %Ending with the final path point
    
    figure();
    plot(run_points(:, 1), run_points(:, 2), 'b');
    hold on
    gscatter(change_points(:, 1), change_points(:, 2), cluster);
    %plot(change_points(:, 1), change_points(:, 2) )
    hold on
    plot(cluster_boundaries(:, 1), cluster_boundaries(:, 2), '*' );
    axis equal
end
