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
b = false
while(a)
    if b || isequal(change_points(8,:), run_points(i,:))
        first = [first, run_points(i, :)'];
        b = true;
    end
    if isequal(change_points(13,:), run_points(i,:))
        first = [first, run_points(i, :)'];
        break;
    end
    i = i+1;
end
%%
k = 3; %degree
%t = [0 0 0 0 0 0.25 0.5 0.60 0.75 0.80 1 1 1 1 ]
t = linspace(0, 1, length(first(1,:)) + k);
 [M_0,x] = bspline_deboor(k,t,first, 100);

 
 figure;
hold all;
plot(first(1,:), first(2,:), 'r');
plot(M_0(1,:), M_0(2,:), 'b*');
legend('true control points', 'original data', ...
    'Location', 'Best');
hold off;

%% código do stor
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
figure();
plot(P(1,:), P(2,:), 'r');
hold on;
plot(xx,yy);


