close all
clear;
clc;
%%
load('../../occupancyMatrix.mat', 'occupancyMatrix');
load('../../run_points.mat', 'run_points');

 [change_points, cluster, threshold] = pathSegmentation(run_points);
 save('../mat_files/segmentation.mat', 'change_points', 'cluster');

P = run_points'; 
%only for the first cluster testing
first = [];
a = true;
i = 1;
b = false;
cluster_id = 0;
last_cluster = max(find(cluster==cluster_id));
M = [];
while(i <  length(run_points(:,1)))
    %find the cluster
    if i == 11
        disp('i = 11')
    end
    if isequal(run_points(i,:), change_points(last_cluster, :)) || b
        b = true;
        if round(norm(run_points(i,:) - change_points(last_cluster,:))) > threshold/2
            first = [first, run_points(i,:)'];
            k = 3; %degree
            x = linspace(0, 1, length(first(1,:)) + k);
            y = gaussmf(x, [0.1 0.5]);
            t = cumsum(y)/max(cumsum(y));
            [M_0,x] = bspline_deboor(k,t,first, 100);
            M = [M, M_0];
            first = [M_0(: , end)];
            cluster_id = cluster_id + 1;
            last_cluster = max(find(cluster==cluster_id));
            b = false;
            continue;
        end
    end
    first = [first, run_points(i,:)'];
    i = i +1;
end
%%
figure();
hold all;
plot(run_points(:,1), run_points(:,2), 'r');
plot(M(1,:), M(2,:), 'b');
legend('Dijsktra Path', 'Smoothed Path', ...
    'Location', 'Best');
title("Our way");
axis equal
hold off;

% %% código do stor
% x = P(1, :);
% y = P(2, :);
% h = 0.01;
% npt = length(x);        % number of via points, including initial and final
% nvia = [0:1:npt-1];
% csinterp_x = csapi(nvia,x);
% csinterp_y = csapi(nvia,y);
% time = [0:h:npt-1];
% xx = fnval(csinterp_x, time);
% yy = fnval(csinterp_y, time);
% figure(2);
% plot(P(1,:), P(2,:), 'r');
% hold on;
% plot(xx,yy);
% title('Professor way')

function  [change_points, cluster, threshold] = pathSegmentation(run_points)
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

