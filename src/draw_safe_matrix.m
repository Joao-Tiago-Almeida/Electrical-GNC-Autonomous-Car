function safe_matrix = draw_safe_matrix(occupancyMatrix, meters_from_MAP, safe_distance, forbidden_zone)

if nargin < 1
    load('../mat_files/occupancyMatrix.mat', 'occupancyMatrix');
    meters_from_MAP = 0.1762;   % meters/pixel
    safe_distance = 1.5;    % meters
    forbidden_zone = 0.64;  % meters
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
func_stable_region = @(x) A*log(x+B).*(forbidden_pixels<=x).*(x<=safe_pixels) + inf_limit*(x>safe_pixels);

%   input vector to convulution
radius = ceil(safe_pixels);
v_up = 0:radius;
v = [v_up flip(v_up)];
[X,Y] = meshgrid(v,v);
u = func_stable_region(sqrt(2*(radius).^2) - sqrt((radius-X).^2+(radius-Y).^2));

figure('WindowStyle', 'docked');
hold on
grid on
fplot(func_stable_region);
title("Convulution function");
xlabel("[meters]")
ylabel("weigth")
xlim([0 length(v_up)])
ylim([0 inf_limit])
plot(v_up(1):length(v_up), func_stable_region(v_up(1):length(v_up)), 'ro');

figure('WindowStyle', 'docked');
mesh(u);
xlabel("pixels")
ylabel("pixels")
zlabel("weigth")

%   evaluates the danger zones
Ch = conv2(occupancyMatrix, u, 'same');

%   normalize to 1 byte
normalize = 255/max(max(Ch));
safe_matrix = round(Ch*normalize .* occupancyMatrix);
save('../mat_files/safe_matrix.mat', 'safe_matrix');

% view
f=figure('WindowStyle', 'docked');
mesh(flip(safe_matrix), 'EdgeColor', 'interp', 'FaceColor', 'interp');
%view(0,90)
axis equal
axis tight
axis manual
title("Potential Danger Zones", "FontSize", 20, "FontName", "arial")
colormap jet

end