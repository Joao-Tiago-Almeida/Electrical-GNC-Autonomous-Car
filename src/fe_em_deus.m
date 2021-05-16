close all
clear;
clc;
%%
load ('../../run_points.mat', 'run_points')
load('../mat_files/segmentation.mat', 'change_points', 'cluster')

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
    if isequal(run_points(i,:), change_points(last_cluster, :))
        first = [first, run_points(i,:)'];
        k = 4; %degree
        %t = [0 0 0 0 0 0.25 0.5 0.60 0.75 0.80 1 1 1 1 ]
        t = linspace(0, 1, length(first(1,:)) + k);
        [M_0,x] = bspline_deboor(k,t,first, 100);
        M = [M, M_0];
        first = [];
        cluster_id = cluster_id + 1;
        last_cluster = max(find(cluster==cluster_id));
    end
    first = [first, run_points(i,:)'];
    i = i +1;
end
%%

    
figure(1);
hold all;
plot(run_points(:,1), run_points(:,2), 'r');
plot(M(1,:), M(2,:), 'b');
legend('true control points', 'original data', ...
    'Location', 'Best');
title("Our way");
hold off;

%% c�digo do stor
x = P(1, :);
y = P(2, :);
h = 0.01;
npt = length(x);        % number of via points, including initial and final
nvia = [0:1:npt-1];
csinterp_x = csapi(nvia,x);
csinterp_y = csapi(nvia,y);
time = [0:h:npt-1];
xx = fnval(csinterp_x, time);
yy = fnval(csinterp_y, time);
figure(2);
plot(P(1,:), P(2,:), 'r');
hold on;
plot(xx,yy);
title('Professor way')


