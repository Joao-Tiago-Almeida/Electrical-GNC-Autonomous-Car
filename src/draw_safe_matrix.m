function safe_matrix = draw_safe_matrix(binaryMatrix, meters_from_MAP, safe_distance)

if nargin < 1
    load('binaryMatrix.mat', 'binaryMatrix');
    meters_from_MAP = 0.1762;   % meters/pixel
    safe_distance = 2.5;    % meters
end

%   ideal distance to preserve from obstacles
safe_pixels = safe_distance/meters_from_MAP;

%   evaluates the potential risk from to the forbidden zone
func_stable_region = @(x) log(x.*safe_pixels);

%   input vector to convulution
u = 1:safe_pixels;
u = func_stable_region(u);

%   evaluates the danger zones
Ch = conv2(u,u',binaryMatrix);

%   normalize to 1 byte
normalize = 255/max(max(Ch));
safe_matrix = round(Ch*normalize);
save('safe_matrix.mat', 'safe_matrix');

% view
mesh(flip(safe_matrix));
%view(0,90)
axis equal
axis tight
axis manual
title("Potential Danger Zones", "FontSize", 20, "FontName", "arial")

end