function safe_matrix = draw_safe_matrix(occupancyMatrix, meters_from_MAP, safe_distance, forbidden_zone)

if nargin < 1
    load('../mat_files/occupancyMatrix.mat', 'occupancyMatrix');
    meters_from_MAP = 0.1762;   % meters/pixel
    safe_distance = 2.5;    % meters
    forbidden_zone = 1.7;  % meters
end

% get occupancy values
occupancyMatrix(occupancyMatrix > 0) = 1;

% ideal distance to preserve from obstacles
safe_pixels = safe_distance/meters_from_MAP;
forbidden_pixels = forbidden_zone/meters_from_MAP;

% function speacifications
inf_limit = 1;
B = 1 - forbidden_pixels;
A = inf_limit/(log(safe_pixels+B));

%   evaluates the potential risk from to the forbidden zone
func_stable_region = @(x) A*log(x+B).*(forbidden_pixels<=x).*(x<=safe_pixels) + inf_limit*(x>=safe_pixels);

%   input vector to convulution
radius = ceil(safe_pixels);
v_up = 0:radius;
v = [v_up flip(v_up)];
[X,Y] = meshgrid(v,v);
u = func_stable_region(sqrt(2*(radius).^2) - sqrt((radius-X).^2+(radius-Y).^2));

%% view
close all
f1 = figure('WindowStyle', 'docked');
%f1.Position(4)=f1.Position(4)/2;
hold on
grid on
fplot(func_stable_region,"LineWidth",2);
title("Convulution function");
xlabel("meters")
ylabel("weigth")
xlim([0 length(v_up)])
ylim([0 inf_limit])
plot(v_up(1):length(v_up), func_stable_region(v_up(1):length(v_up)), 'ro',"MarkerSize",8);
xticklabels(num2cell(xticks*meters_from_MAP))

f2=figure('WindowStyle', 'docked');
%f2.Position(4)=f2.Position(4)/2;
mesh(u,'FaceColor', 'flat');
light
% lighting gouraud
title("3D Convulution function");
xlabel("pixels")
ylabel("pixels")
zlabel("weigth")

%   evaluates the danger zones
Ch = conv2(occupancyMatrix, u, 'same');
max_value=255;
normalize = max_value/max(max(Ch));
safe_matrix = round(Ch*normalize .* occupancyMatrix);
safe_matrix(safe_matrix<max_value/2)=0;
save('../mat_files/safe_matrix.mat', 'safe_matrix');

f3=figure('WindowStyle', 'docked');
%f3.Position(4)=f3.Position(4)/2;
mesh(safe_matrix, 'EdgeColor', 'interp', 'FaceColor', 'flat');
axis equal
axis tight
axis manual
view(90,-90)
title("Potential Danger Zones", "FontSize", 20, "FontName", "arial")
xlabel("pixels")
ylabel("pixels")
colormap jet
cb=colorbar;
cb.Ticks = [0 255];
cb.TickLabels=["Circuit Limitation","Safe Area"];
cb.FontSize=14;
cb.Location="South";
cb.Position(2)=0.18;
xlim([300 900])
ylim([100 1400])
end