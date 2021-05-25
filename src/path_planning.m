function [smoothed_path, checkpoints] = path_planning(path_points)
% This function is responsabile to planning a path taking in account the
% points {start, middle, stop}. It uses an dynamical weight dijkstra
% algorithm. This version migth computes two different path in order to
% minimize the time or distance.

    %% Define variables to be used throughout the file
    global occupancy_matrix map_information...
        map_grid points...
        points_grid gap_between_cells dx dy ...
        yx_2_idx_graph idx_graph_2_xy ...
        m_occupancy m_safe  ...
        node_location heap directions ...
        debug_mode file_path
    
    % to be returned
    smoothed_path = [];
    checkpoints = [];
    
    % a path need 2 points
    if(size(path_points,1)<2); return; end
  
    if(isempty(occupancy_matrix)); load(string(file_path+"occupancy_matrix.mat"), 'occupancy_matrix'); end
    if(isempty(map_information)); map_information=load(string(file_path+"mapInformation.mat")); end
    if(isempty(path_points)); load(string(file_path+"path_points.mat"),'path_points'); end
    
    safe_matrix = draw_safe_matrix;
    gap_between_cells = 5;
    compute_map_grid(path_points);

    % convert the points in the occupancy matrix in the grid
    points_grid(:,1) = (points(:,1)-1)/gap_between_cells+1;
    points_grid(:,2) = (points(:,2)-1)/gap_between_cells+1;

    % cell divisions
    dx = size(map_grid,2);
    dy = size(map_grid,1);

    % mapping between grid access and 1D vect idx
    yx_2_idx_graph = @(y,x) dx*(y-1)+x;
    idx_graph_2_xy = @(idx) [rem(idx,dx) (idx-rem(idx,dx))/dx+1];

    % change to symmatric matrix since it is a minimization problem
    occupancy_matrix(occupancy_matrix==2)=1;  % ignoring crosswalks
    occupancy_matrix(occupancy_matrix==3)=0.9;% slown down on traffic lights (the less, the lighter)
    occupancy_matrix(occupancy_matrix==4)=0;  % stop in Stop sings
    m_occupancy = 1-occupancy_matrix;
    m_safe = 1-safe_matrix/max(max(safe_matrix));

    %% Auxiliar Structs
    heap = MinHeap(sum(sum(map_grid))-1);
    T = arrayfun(@(~) struct('cost',[],'distance',[],'index',[],'previous',[],'linear_velocity',[],...
                                'angular_velocity',[]), 1:size(map_grid,1)*size(map_grid,2), 'UniformOutput',false);
    node_location = horzcat(T{:});
    directions = struct('N',[0 -1],'NE',[1 -1],'E',[1 0],'SE',[1 1],'S',[0 1],'SW',[-1 1],'W',[-1 0],'NW',[-1 -1],...
                        'idxs' ,[0 -1;1 -1;1 0;1 1;0 1;-1 1;-1 0;-1 -1],...
                        'names', ["N","NE","E","SE","S","SW","W","NW"]);

    %% DYNAMIC DIJKSTRA - Path planning
    %https://media.neliti.com/media/publications/165891-EN-shortest-path-with-dynamic-weight-implem.pdf

    sub_path = [];  % last confirmed path but it migth not occured if the stop point has to be deleted 
    path_data = []; % accumulated path with start and stop points confirmed
    prev_node=[];   % information about the start point 
    prev_prev_node=[];  % information about the start point before
    i=1;
    valid_points = 1:size(points_grid,1);
    wb=waitbar(0,"Planning The Best Path");
    wb.Position(1)= wb.Position(1)-wb.Position(3);
    tic
    while(i<length(valid_points))
        start = points_grid(valid_points(i),:);
        idx_start = yx_2_idx_graph(start(2),start(1));
        stop = points_grid(valid_points(i+1),:);
        idx_stop = yx_2_idx_graph(stop(2),stop(1));

        wb=waitbar((i-1)/length(valid_points),wb,"Planning sub Path "+num2str(i));
        dijkstra(idx_start,idx_stop,prev_node,"time");

        if(isempty(node_location(idx_stop).cost))
            valid_points(i)=[];
            i=i-1;
            prev_node = prev_prev_node;
            sub_path = [];
            disp("Invalid Subpath")
        else
            path_data = [path_data;sub_path];
            sub_path = get_path(idx_start,idx_stop);
            prev_prev_node = prev_node;
            prev_node = node_location(idx_stop);
            i=i+1;
        end

        node_location = horzcat(T{:});
        heap.Clear();
    end
    toc
    delete(wb);

    if(isempty(sub_path))
        disp("No Path available");
        return
    end

    path_data = [points_grid(1,:),zeros(1,size(sub_path,2)-2);    path_data;  sub_path];

    %% Path analysis
    max_velocity=30; %Km/h
    n_points = length(path_data);
    path_distance = gap_between_cells*path_data(end,5)*map_information.meters_from_MAP;
    mean_velocity = max_velocity*sum(path_data(:,4))/n_points; 
    path_duration = 3.6*path_distance/mean_velocity;    % 1m/s = 3.6 Km/mh
    average_velocity = 3.6*gap_between_cells*norm(points_grid(1,:)-points_grid(end,:))*map_information.meters_from_MAP/path_duration;

    disp(" /'----------Path-Planning-----------'\")
    %disp("| Points:             "   +num2str(n_points)             +  "     pixels.  |")
    disp("| Distance:           "   +num2str(path_distance,"%.2f")   +  "  meters.  |")
    disp("| Duration:           "   +num2str(path_duration,"%.2f")   +  "   seconds. |")
    disp("| Mean Velocity:      "   +num2str(mean_velocity,"%.2f")   +  "   Km/h.    |")
    disp("| Average Velocity:   "   +num2str(average_velocity,"%.2f")+  "   Km/h.    |")
    disp(" \,----------------------------------,/")
    
    %% transpose to the original map
    run_points = [(path_data(:,1)-1)*gap_between_cells+1 (path_data(:,2)-1)*gap_between_cells+1];
    save(string(file_path+"run_points.mat"), 'run_points');
    
    % there are not suficient number of points
    if(length(valid_points)<2);return;end

    %% validate checkpoints
    checkpoints=points(valid_points,:);
    save(string(file_path+"checkpoints.mat"), 'checkpoints');    
    
    %% Path Smoothing
    fig = uifigure;
    d=uiprogressdlg(fig,'Title','Smoothing the Path','Indeterminate','on');
    drawnow
    smoothed_path = path_smoothing(run_points,checkpoints,map_information.meters_from_MAP);
    close(d);close(fig);
    
    
    %% Final plots and verifications
    if(debug_mode)
        inspect_plots(smoothed_path, run_points, checkpoints, path_data, n_points, path_duration, max_velocity)
        disp("[EOF] Path Planning")
        license('inuse')
        %[fList,pList] = matlab.codetools.requiredFilesAndProducts('path_planning.m');
    end

end

%% Visual Support Content
function inspect_plots(smoothed_path, run_points, checkpoints, path_data, n_points, path_duration, max_velocity)
% This functions displays in figures the path planned
    
    global map_information file_path
    MAP = load(string(file_path+"MAP.mat"),'MAP');
    MAP.MAP.Name = 'Path Planning Velocity';
    hold on
    plot(checkpoints(:,1),checkpoints(:,2),"kd","LineWidth",6)
    patch([run_points(:,1);NaN],[run_points(:,2);NaN],[path_data(:,4);NaN],...
        [max_velocity*path_data(:,4);NaN],'EdgeColor','interp',"Linewidth",4);
    cb=colorbar;
    cb.TickLabels=cb.TickLabels+" Km/h";
    cb.Position = [0.91 0.05 0.02 0.9];
    colormap(jet);
    Image = getframe(gcf);
    imwrite(Image.cdata, string(file_path+"dijkstra_path.png"), 'png');
    place_car(run_points,10)
    
    
    figure('WindowStyle', 'docked');
    t=0:seconds(path_duration)/(n_points-1):seconds(path_duration)';

    subplot(2,1,1)
    hold on
    title("Run Velocity per distance")
    plot(path_data(:,5),path_data(:,4));
    xlim([0 path_data(end,5)])
    
    subplot(2,1,2)
    hold on
    title("Run Cost")
    plot(t,path_data(:,3));
    xlim([0 seconds(path_duration)])
    
    % image is not in the real World
    if(length(fieldnames(map_information))~=3)
        return
    end

    figure('WindowStyle', 'docked');
    lat = map_information.fget_Lat_from_MAP(smoothed_path(:,2));
    lon = map_information.fget_Lon_from_MAP(smoothed_path(:,1));
    geoplot(lat,lon,'g-*')

    %% Web map
    % webmap
    % wmline(lat,lon)
    % wmmarker(map_information.fget_Lat_from_MAP(smoothed_path(:,2)),map_information.fget_Lon_from_MAP(smoothed_path(:,1)))

end

%% Auxiliar functions to Path Planning
function dijkstra(idx_start,idx_finish,init_node,loss_criterium)
% Dynamical Weight Dijkstra where it is computed a path between two pixels,
% given priority two straight movement.
% It writes in the variable node_location the reachable points, and the
% algorithm stops either when the end point is reached or when there are
% not any good option to complete the path.

    global heap directions idx_graph_2_xy yx_2_idx_graph node_location
    
    in_heap = zeros(length(node_location),1);

    % Starting from the initial point
    if(isempty(init_node))
        heap.InsertKey(new_node(0,0,idx_start,'',0,0))
    else
        heap.InsertKey(init_node);
    end
    in_heap(idx_start) = 1;
    
	reach_endpoint = false;
    
    wb=waitbar(0,"Closiness of the destination");
    % Do Dijkstra
    while(~reach_endpoint && heap.Count()>0)
        
        node = heap.ExtractMin();
        node_location(node.index)=node;
        in_heap(node.index) = -1;
        
        % Confirm if the end point was reached
        if(idx_finish==node.index)
            %disp("Computed points: "+num2str(sum(in_heap==-1)))
            delete(wb);
            return
        end
        waitbar(1-norm(idx_graph_2_xy(node.index)-idx_graph_2_xy(idx_finish))/...
            (norm(idx_graph_2_xy(idx_start)-idx_graph_2_xy(idx_finish))),...
            wb,...
            "Closiness of the destination");
        
        reachable_neighbours = identify_reachable_neighbours(idx_graph_2_xy(node.index),node.previous, sum(in_heap==-1)==1);

        for i = 1:8% arround movements
            if reachable_neighbours(i) == 0;continue;end
            
            step = directions.idxs(i,:);
            new_pos = idx_graph_2_xy(node.index)+step;
            
            % new node information
            new_idx = yx_2_idx_graph(new_pos(2),new_pos(1));
            direction=directions.names(i);
            [linear_velocity,angular_velocity] = compute_velocity(node.previous,directions.names(i),node.linear_velocity,node.angular_velocity,(directions.names(i)~=node.previous),new_pos);
            distance = node.distance+(directions.names(i)==node.previous)+(directions.names(i)~=node.previous)*sqrt(2); % Since it it normalised to [0 1] it is equivalent is is a true statement           
            cost = compute_cost(new_pos,linear_velocity,angular_velocity,distance,loss_criterium,node.cost);
            
            % Define the new atempt
            node_aux = new_node(cost,distance,new_idx,direction,linear_velocity,angular_velocity);
                
            % The node is new to the heap
            if(in_heap(node_aux.index) == 0)
                heap.InsertKey(node_aux);
                in_heap(node_aux.index) = 1;
            % The node is already in the heap
            elseif(in_heap(node_aux.index) == 1)
                heap.UpdateElement(node_aux);
            end
        end
    end
    delete(wb);
end

function subpath = get_path(idx_start,idx_finish)
% Reconstructs the path, starting from the finish point and move to the
% previous until reach the first. When a path is not valid, it recomputes a
% path from the previous directly to the next checkpoint

    global node_location directions yx_2_idx_graph idx_graph_2_xy
    idx = idx_finish;
    subpath = zeros(0,5);
    
    % Could not reach the goal
    if(isempty(node_location(idx).cost))
        return
    end
    
    while(idx ~= idx_start)
        % add the last point
        subpath = [idx_graph_2_xy(idx),node_location(idx).cost,...
                   node_location(idx).linear_velocity,node_location(idx).distance;
                    subpath];
        
        % frontwards movement
        [~,I_front] = max(strcmp(directions.names,node_location(idx).previous)); 
        
        % retrocede 1 movement
        step = directions.idxs(I_front,:);
        new_pos = idx_graph_2_xy(idx) - step;
        new_idx = yx_2_idx_graph(new_pos(2),new_pos(1));
        
        idx = node_location(new_idx).index;
    end
end

function reachable_neighbours = identify_reachable_neighbours(xy,previous_direction,first_point)
% This function computes the points that are considered to be a next move.
% When it is the start point, the can move all over the place , however for
% any other circunstance, the car only of three choices: straight, left or
% rigth diagonal (degrees).

    global directions map_grid dx dy
   
   points = xy+directions.idxs;
   in_boundaries = logical(sum((points<=[dx dy]).*(points>=1),2)==2);
   points(~in_boundaries,:) = 1;
   
   reachable_neighbours = diag(map_grid(points(:,2),points(:,1))).*in_boundaries;  % at this point it contains all 
   idxs = ones(1,8);
    if(previous_direction ~= '')
        % When the previous direcition is one of {N;E;S;W}, the next
        % direction has to contain this letter in the direction name
        % once the available movements are withing [-45,45]degrees.
        if(strlength(previous_direction)==1)
            idxs = contains(directions.names,previous_direction);
        
        else
            previous_direction = char(previous_direction);
            idxs = strcmp(directions.names,previous_direction)+...
                strcmp(directions.names,previous_direction(1))+...
                strcmp(directions.names,previous_direction(2));
        end
   end
    
   reachable_neighbours = logical(reachable_neighbours.*idxs');
   
%    if it is the first point and does not have no option, go back
   if(first_point && sum(reachable_neighbours)==0)
       [~,idx_front] = max(strcmp(directions.names,previous_direction));
       idx_back = rem(idx_front+4,8); idx_back(idx_back==0)=8;
       reachable_neighbours(idx_back)=true;
   end
   
   % Availbale directions: directions.names(logical(reachable_neighbours))
end

function cost = compute_cost(end_pos,linear_velocity,angular_velocity,distance,loss_criterium,prev_cost)
% This function selects the loss function that is used to compute the
% cost of each node as well as its cost on the safetiness matrix
    
    global m_safe gap_between_cells
    
    if (loss_criterium=="time")
        loss = prev_cost+(1-linear_velocity)^2+(1+angular_velocity)^2;
    else % (loss_criterium=="distance")
        loss=distance;
    end
    
    %% This section takes into account also the cost of being in certain parts of the road, do not change
    cost = loss*(1+m_safe(1+gap_between_cells*(end_pos(2)-1),1+gap_between_cells*(end_pos(1)-1)));
end

function [linear_velocity,angular_velocity] = compute_velocity(start_dir,end_dir,prev_linear_velocity,prev_angular_velocity,change_of_direction,end_pos)
% Thsi function simulates the velocity towards the time taking into
% consideration changes in direction

    global m_occupancy gap_between_cells map_information
    
    if (prev_linear_velocity<0.05)            % the car was stop in the last moment
        linear_velocity = 0.05;
    elseif (start_dir ~= end_dir)               % 45 degrees curve
    	linear_velocity = prev_linear_velocity*cos(pi/4);
    else                                        % straight movement
        velocity_increment = 1+map_information.meters_from_MAP*gap_between_cells/10;
        linear_velocity = prev_linear_velocity*velocity_increment;
    end
     
    % increment velocity every iteration
   
    linear_velocity = min(1,linear_velocity);
    
    % relieve the steering wheel after each change
    angular_velocity = change_of_direction*(prev_angular_velocity+pi/4);
    angular_velocity = 0.5*angular_velocity;
    
    % weights traffic light ans stop zones
    linear_velocity = linear_velocity*(1-m_occupancy(1+gap_between_cells*(end_pos(2)-1),1+gap_between_cells*(end_pos(1)-1)));
end

function s = new_node(c,d,i,p,lv,av)
% Struct of the node used in the heap

    % cost to reach the node
    f_cost = 'cost';
    v_cost = c;
        
    % accumulated distance
    f_distance = 'distance';
    v_distance = d;
    
    % index in the graph
    f_index = 'index';
    v_index = i;
    
    % previous node
    f_previous = 'previous';
    v_previous = p;
    
    % linear velocity
    f_linear_velocity = 'linear_velocity';
    v_linear_velocity = lv;
    
    % angular velocity
    f_angular_velocity = 'angular_velocity';
    v_angular_velocity = av;
    
    s = struct(f_cost,v_cost,f_distance,v_distance,f_index,v_index,f_previous,v_previous,...
        f_linear_velocity,v_linear_velocity,f_angular_velocity,v_angular_velocity);
end

%% Safetiness Matrix
function safe_matrix = draw_safe_matrix(safe_distance, forbidden_zone)
% This function computes the safetiness matrix where it weights the
% neighbourhood and classifies whether is safe to drive in that zone or not

    global occupancy_matrix map_information debug_mode file_path
    
    if nargin < 1
        safe_distance = 2.5;    % meters
        forbidden_zone = 1.7;  % meters
    end
    meters_from_MAP = map_information.meters_from_MAP;   % meters/pixel

    % get occupancy values
    occupancy_matrix(occupancy_matrix > 0) = 1;

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
    
    
    %   evaluates the danger zones
    Ch = conv2(occupancy_matrix, u, 'same');
    max_value=255;
    normalize = max_value/max(max(Ch));
    safe_matrix = round(Ch*normalize .* occupancy_matrix);
    safe_matrix(safe_matrix<max_value/2)=0;
    save(string(file_path+"safe_matrix.mat"), 'safe_matrix');
    
    if(~debug_mode);return;end
    
    %% view
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
    mesh(u,'FaceColor', 'flat');
    light
    % lighting gouraud
    title("3D Convulution function");
    xlabel("pixels")
    ylabel("pixels")
    zlabel("weigth")

    f3=figure('WindowStyle', 'docked');
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

%% Visibility Matrix
function compute_map_grid(path_points)
% It is computed a visibility matrix from the occupancy grid where the gap
% between the cells is the space of each division

    global occupancy_matrix gap_between_cells points map_grid debug_mode file_path
    [dim_y, dim_x] = size(occupancy_matrix);

    dx = 1:gap_between_cells:dim_x;
    dy = 1:gap_between_cells:dim_y;
    [X,Y] = meshgrid(dx,dy);
    map_grid = occupancy_matrix(dy,dx)~=0;
    [points,~] = get_closest_point_in_grid(gap_between_cells,gap_between_cells,path_points,occupancy_matrix);
    
    if(~debug_mode);return;end
    
    f_aux = figure('WindowStyle', 'docked', 'Units' ,'pixel');
    mesh(X,Y,map_grid);
    pbaspect([1 1 1]);
    hold on
   
    plot(points(:,1), points(:,2), 'ro')
    view(0,-90);

    xlim([1 dx(end)]);
    ylim([1 dy(end)]);
    
    Image = getframe(gcf);
    imwrite(Image.cdata, string(file_path+"grid_w_points.png"), 'png');
    close(f_aux)

    % plot new image
    %f_aux=figure;
    %I = imread(string(file_path+"grid_w_points.png');
    %imshow(I);
    %f_aux.WindowStyle='docked';
end

function [points,allowed_points] = get_closest_point_in_grid(nx,ny,xy,occupancy_matrix)
% Place the point selected by the user in the nearest point of the
% visibility matrix
    global file_path
    
    points = [];    %   vector of neighbours in the grid
    allowed_points = zeros(size(xy,1),1);     %   real points inside the matrix
    %   float numbers inside the square
    rem_x = rem(xy(:,1),nx);
    rem_y = rem(xy(:,2),ny);
    
    %   The four corners of the closest square
    upper_left_corner = [xy(:,1)-rem_x+1, xy(:,2)-rem_y+1];
    upper_right_corner = [upper_left_corner(:,1)+nx, upper_left_corner(:,2)];
    lower_left_corner = [upper_left_corner(:,1) upper_left_corner(:,2)+ny];
    lower_right_corner = [upper_left_corner(:,1)+nx upper_left_corner(:,2)+ny];
    
    % iterate over the corners
    for idx = 1:size(upper_left_corner,1)
        best_corner = [upper_left_corner(idx,:)
                       upper_right_corner(idx,:)
                       lower_left_corner(idx,:)
                       lower_right_corner(idx,:)];

        dist = vecnorm(xy(idx,:)-best_corner,2,2);    % euclidian distance to the corners: (vec, euclidian norm, dim)
        
        [~,I] = sort(dist); % closet neighbour
        
        % verify if the point is inside the road
        for c = 1:length(I)
            if(occupancy_matrix(best_corner(I(c),2), best_corner(I(c),1)) ~= 0)    % point inside the road
                points = [points;best_corner(I(c),:)];
                allowed_points(idx) = 1;
                break
            end
        end
    end

    if(sum(allowed_points) == 0)
        disp("There any any valid points")
    elseif(sum(allowed_points) == 1)
        disp("There are only one available point")
        points = [];
    end
    
    save(string(file_path+"path_points_grid.mat"), 'points');
end