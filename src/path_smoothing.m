function sampled_path = path_smoothing(run_points,checkpoints,meters_from_MAP)
    
    global debug_mode file_path
    run_points = insert_checkpoints_in_runPoints(run_points, checkpoints);
    [change_points, cluster, cluster_boundaries] = path_segmentation(run_points, checkpoints, meters_from_MAP);

    smoothed_path = spline_clusters(change_points, cluster, cluster_boundaries, checkpoints);
    smoothed_path = [checkpoints(1,:); smoothed_path'; checkpoints(end,:)];
    
    fixed_sample_rate = 0.5; % meters
    sampled_path = resample_path(smoothed_path, meters_from_MAP, fixed_sample_rate);
    save(string(file_path+"sampled_path_"+num2str(fixed_sample_rate)+"_meters.mat"),'sampled_path');
    
    max_velocity=30; %Km/h
    velocity = max_velocity*compute_velocity(sampled_path, fixed_sample_rate);
    
    %% Path analysis
    path_distance = sum(meters_from_MAP*vecnorm(diff(sampled_path) ,2,2));
    mean_velocity = mean(velocity); 
    path_duration = 3.6*path_distance/mean_velocity;    % 1m/s = 3.6 Km/mh
    average_velocity = 3.6*norm(checkpoints(1,:)-checkpoints(end,:))*meters_from_MAP/path_duration;
    
    disp(" /'----------Path-Smoothing----------'\")
    disp("| Distance:           "   +num2str(path_distance,"%.2f")   +  "  meters.  |")
    disp("| Duration:           "   +num2str(path_duration,"%.2f")   +  "   seconds. |")
    disp("| Mean Velocity:      "   +num2str(mean_velocity,"%.2f")   +  "   Km/h.    |")
    disp("| Average Velocity:   "   +num2str(average_velocity,"%.2f")+  "   Km/h.    |")
    disp(" \,----------------------------------,/")
    
    %% Final plots and verifications
    
    if debug_mode
        
        figure('WindowStyle', 'docked');clf;   
        gscatter(change_points(:, 1)*meters_from_MAP, change_points(:, 2)*meters_from_MAP,cluster);
        hold on
        axis equal
        plot(run_points(:, 1)*meters_from_MAP, run_points(:, 2)*meters_from_MAP, 'r--');
        plot(smoothed_path(:,1)*meters_from_MAP, smoothed_path(:,2)*meters_from_MAP, 'b-')
        plot(sampled_path(:,1)*meters_from_MAP, sampled_path(:,2)*meters_from_MAP, 'k.')
        
        legend(["Cluster " + unique(cluster)',...
            'Dijsktra Path',...
            'Smoothed Path',...
            "Fixed Rate Path at "+num2str(fixed_sample_rate)+" m"],...
            'Location', 'Best');
        xlabel("meters")
        ylabel("meters")
        % plot the final path in the original map
        
        MAP = load(string(file_path+"MAP.mat"),'MAP');
        MAP.MAP.Name = 'Path Smoothing Velocity';
        plot(checkpoints(:,1),checkpoints(:,2),"wd","LineWidth",6)
        hold on
        patch([sampled_path(:,1);NaN],[sampled_path(:,2);NaN],[velocity;NaN],...
            [velocity;NaN],'EdgeColor','interp', 'LineWidth', 4);
        cb=colorbar;
        cb.TickLabels=cb.TickLabels+" Km/h";
        cb.Position = [0.91 0.05 0.02 0.9];
        colormap(jet);
        Image = getframe(gcf);
        imwrite(Image.cdata, string(file_path+"bspline_path.png"), 'png');
    end
    place_car(sampled_path,1);
    
end


function velocity = compute_velocity(sampled_path, fixed_sample_rate)
% This function computes the velocity during the path
    global m_occupancy
    
    interval_between_points = diff(sampled_path);
    angles = atan2(interval_between_points(:, 2),interval_between_points(:,1));
    angle_variation = diff(angles);
    
    velocity = zeros(length(sampled_path),1);
    velocity(1) = 0; %initial
    velocity(2) = 0.05; %initial
    for current_loss = 1:length(angle_variation)
        % curve 
        if cos(angle_variation(current_loss)) < 1
            velocity(2+current_loss) = cos(angle_variation(current_loss))*velocity(1+current_loss);
        % straigth
        else
            velocity(2+current_loss) = min(1, cos(angle_variation(current_loss))*(1+(fixed_sample_rate/10))*velocity(1+current_loss));
        end
        % take into account the road conditions
        velocity(2+current_loss) = velocity(2+current_loss)*m_occupancy(round(sampled_path(3,2)),round(sampled_path(3,1)));
    end
end

function run_points = insert_checkpoints_in_runPoints(run_points, checkpoints)
    % This function replaces in the planned path (run_points) the NEAREST
    % point to a check point with that checkpoint. 
    % This is needed because information is lost by replacing the
    % image with a grid, so this is to force the path to have the CheckPoints!
    %
    % Input Arguments:
    %
    % run_points - points from the path planning algorithm
    % check_points - check points from the user input
    
    for current_checkpoint_index = 2:size(checkpoints,1)-1
        norm_points = vecnorm( run_points - checkpoints(current_checkpoint_index,:), 2, 2 );
        [~, closestpoint_index] = min(norm_points);
        run_points(closestpoint_index,:) = checkpoints(current_checkpoint_index,:);
    end
end

function  [change_points, clusters, cluster_boundaries] = path_segmentation(run_points, checkpoints, meters_from_MAP)
    % This function segmentates the path in sections
    % Input Arguments:
    % 
    % run_points: path planned points
    % check_points: check points
    % meters_from_MAP: how much meters a pixel of the map has
    %
    % Output Arguments:
    % change_points: coordinates of the points of the run_points, where the
    %                       car changes its direction;
    % clusters: sets of change_points that make up a cluster (a cluster
    %               has points with the same number and the index 
    %               of a point in the cluster is the same index in the
    %               change_points);
    % cluster_boundaries: points that delimeter the clusters;
    
    %computing the change of directions
    interval_between_points = diff(run_points);
    theta = atan2(interval_between_points(:, 2),interval_between_points(:,1));
    diff_theta = diff(theta);
    diff_theta(diff_theta ~=0);
    diff_theta = logical(diff_theta);
    diff_theta = [true; diff_theta; true];
    
    %computing the change_points
    change_points = run_points;
    change_points(~diff_theta,:) = [];
    
    aware_distance = 10; %reasonable distance between points of the same cluster (in meters)
    aware_distance_in_pixels = round(aware_distance/meters_from_MAP);
    clusters = zeros(size(change_points,1),  1);
    counter = 0; %number for each cluster
    for change_point_index = 1:size(change_points,1)-1
        %new cluster
        dist = norm(change_points(change_point_index, :) - change_points(change_point_index+1,:));
        if  dist > aware_distance_in_pixels   
            counter = counter +1;
        end
        clusters(change_point_index+1) = counter;        
    end
    
    cluster_division = diff(clusters); %To check the changes in clusters
    cluster_boundaries = checkpoints(1,:); %Starting with the inital path point
    for point_index = 1:length(cluster_division)
        if cluster_division(point_index)==0
            continue;
        end
            %If is a new cluster
        cluster_boundaries = [cluster_boundaries; ...
                    ( (change_points(point_index,:) + change_points(point_index+1,:)) / 2 ) ];
    end
    %puts the last point
    cluster_boundaries = [cluster_boundaries; ( (change_points(point_index,:) + change_points(point_index+1,:)) / 2 ) ];
    cluster_boundaries(end,:) =  checkpoints(end,:); %Ending with the final path point
   
end

function smoothed_path = spline_clusters(change_points, clusters, cluster_boundaries, checkpoints)
    % Do the b-splines to each cluster to creates the smoothed path
    % Input Arguments:
    %
    % change_points: coordinates of the points of the run_points, where the
    %                       car changes its direction;
    % clusters: sets of change_points that make up a cluster (a cluster
    %               has points with the same number and the index 
    %               of a point in the cluster is the same index in the
    %               change_points);
    % cluster_boundaries: points that delimeter the clusters;
    % check_points - check points from the user input;
    %
    % Output Arguments:
    %
    % smoothed_path: the path after the b-splines
    
    degree = 3; %degree of splines curves
    number_of_clusters = clusters(end)+1;
    smoothed_path = [];
    for current_cluster = 1:number_of_clusters
        all_cluster_points = [(cluster_boundaries(current_cluster,:)); ...
            (change_points( clusters(:)==current_cluster-1,: )); (cluster_boundaries(current_cluster+1,:))];
        
        %Creation of the knot vector t
        x = linspace(0, 1, length(all_cluster_points(:,1)) + degree);
        mu = 0.5;
        sd = 0.5;
        y = 1/(2*pi*sd)*exp(-(x-mu).^2/(2*sd^2));%gaussmf(x, [0.5 0.5]);
        knot_vector = cumsum(y)/max(cumsum(y));
        
        w = ones(1, length(all_cluster_points));
        %give more weight to the checkpoints for the smooth path to get closer
        for chk_point = 1:length(checkpoints)
            norm_points = ~logical (vecnorm( checkpoints(chk_point,:) - all_cluster_points, 2, 2 ));
            w(norm_points) = 3;
        end
        [M_0,~] = bspline_wdeboor(degree,knot_vector,all_cluster_points', w, 100);
        smoothed_path = [smoothed_path, M_0];
    end
end

function resampled_path = resample_path(smoothed_path, meters_from_MAP, fixed_sample_rate)
    % Function to resample the smoothed_path, that is to put consecutive
    % points at the same distance (fixed_sample_rate)
    % 
    % Input Arguments:
    % 
    % smoothed_path: points of the smoothed path;
    % meters_from_MAP: how much meters a pixel of the map has;
    % fixed_sample_rate: distance between the points of the path after the resample
    %
    % Output Arguments:
    % 
    % resampled_path: smoothed_path after the resample
    sampling_dist = fixed_sample_rate/meters_from_MAP;
    current_distance = 0;
    resampled_path = smoothed_path(1,:);
    checkpoint = smoothed_path(1,:);
    for current_point = 1:length(smoothed_path)-1
        current_distance = current_distance + ...
            vecnorm( smoothed_path(current_point,:) -  smoothed_path(current_point+1,:), 2, 2);   
            checkpoint = smoothed_path(current_point,:);
        while(current_distance > sampling_dist)
            %Get slope
            a = smoothed_path(current_point+1,:) - checkpoint;
            slope = atan2(a(2),a(1));
            new_point = checkpoint + sampling_dist*[cos(slope) sin(slope)];
            checkpoint = new_point; %updating checkpoint
            resampled_path = [resampled_path; new_point];
            current_distance = current_distance - sampling_dist;
        end
    end
end

function [C,u] = bspline_wdeboor(n,t,P,w,u)
    % Evaluate explicit weighed B-spline at specified locations.
    %
    % Input arguments:
    % n:
    %    B-spline order (2 for linear, 3 for quadratic, etc.)
    % t:
    %    knot vector
    % P:
    %    control points, typically 2-by-m, 3-by-m or 4-by-m (for weights)
    % w:
    %    weight vector
    % u (optional):
    %    values where the B-spline is to be evaluated, or a positive
    %    integer to set the number of points to automatically allocate
    % Output arguments:
    % C:
    %    points of the B-spline curve
    % Copyright 2010 Levente Hunyadi
    w = transpose(w(:));
    P = bsxfun(@times, P, w);
    P = [P ; w];  % add weights to control points
    if nargin >= 5
        [Y,u] = bspline_deboor(n,t,P,u);
    else
        [Y,u] = bspline_deboor(n,t,P);
    end
    C = bsxfun(@rdivide, Y(1:end-1,:), Y(end,:));  % normalize and remove weights from computed points
end

function [C,U] = bspline_deboor(n,t,P,U)
        % Evaluate explicit B-spline at specified locations.
        %
        % Input arguments:
        % n:
        %    B-spline order (2 for linear, 3 for quadratic, etc.)
        % t:
        %    knot vector
        % P:
        %    control points, typically 2-by-m, 3-by-m or 4-by-m (for weights)
        % u (optional):
        %    values where the B-spline is to be evaluated, or a positive
        %    integer to set the number of points to automatically allocate
        %
        % Output arguments:
        % C:
        %    points of the B-spline curve

        % Copyright 2010 Levente Hunyadi
    validateattributes(n, {'numeric'}, {'positive','integer','scalar'});
    degree = n-1;  % B-spline polynomial degree (1 for linear, 2 for quadratic, etc.)
    validateattributes(t, {'numeric'}, {'real','vector'});
    assert(all( t(2:end)-t(1:end-1) >= 0 ), 'bspline:deboor:InvalidArgumentValue', ...
        'Knot vector values should be nondecreasing.');
    validateattributes(P, {'numeric'}, {'real','2d'});
    nctrl = numel(t)-(degree+1);
    assert(size(P,2) == nctrl, 'bspline:deboor:DimensionMismatch', ...
        'Invalid number of control points, %d given, %d required.', size(P,2), nctrl);
    if nargin < 4
        U = linspace(t(degree+1), t(end-degree), 10*size(P,2));  % allocate points uniformly
    elseif isscalar(U) && U > 1
        validateattributes(U, {'numeric'}, {'positive','integer','scalar'});
        U = linspace(t(degree+1), t(end-degree), U);  % allocate points uniformly
    else
        validateattributes(U, {'numeric'}, {'real','vector'});
        assert(all( U >= t(degree+1) & U <= t(end-degree) ), 'bspline:deboor:InvalidArgumentValue', ...
            'Value outside permitted knot vector value range.');
    end

    number_of_points = size(P,1);  % dimension of control points
    t = t(:).';     % knot sequence
    U = U(:);
    S = sum(bsxfun(@eq, U, t), 2);  % multiplicity of u in t (0 <= s <= degree+1)
    I = bspline_deboor_interval(U,t);

    Pk = zeros(number_of_points,degree+1,degree+1);
    a = zeros(degree+1,degree+1);

    C = zeros(size(P,1), numel(U));
    for control_point = 1 : numel(U)
        u = U(control_point);
        s = S(control_point);
        ix = I(control_point);
        Pk(:) = 0;
        a(:) = 0;

        % identify d+1 relevant control points
        Pk(:, (ix-degree):(ix-s), 1) = P(:, (ix-degree):(ix-s));
        h = degree - s;

        if h > 0
            % de Boor recursion formula
            for r = 1 : h
                q = ix-1;
                for i = (q-degree+r) : (q-s)
                    a(i+1,r+1) = (u-t(i+1)) / (t(i+degree-r+1+1)-t(i+1));
                    Pk(:,i+1,r+1) = (1-a(i+1,r+1)) * Pk(:,i,r) + a(i+1,r+1) * Pk(:,i+1,r);
                end
            end
            C(:,control_point) = Pk(:,ix-s,degree-s+1);  % extract value from triangular computation scheme
        elseif ix == numel(t)  % last control point is a special case
            C(:,control_point) = P(:,end);
        else
            C(:,control_point) = P(:,ix-degree);
        end
    end
end

function ix = bspline_deboor_interval(u,t)
    % Index of knot in knot sequence not less than the value of u.
    % If knot has multiplicity greater than 1, the highest index is returned.

    i = bsxfun(@ge, u, t) & bsxfun(@lt, u, [t(2:end) 2*t(end)]);  % indicator of knot interval in which u is
    [row,col] = find(i);
    [row,ind] = sort(row);  %#ok<ASGLU> % restore original order of data points
    ix = col(ind);
end
