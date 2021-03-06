function [sampled_path, checkpoints] = path_planning(path_points, path_orientation,loss_criterium)
% This function is responsabile to planning a path taking in account the
% points {start, middle, stop}. It uses an dynamical weight dijkstra
% algorithm. This version migth computes two different path in order to
% minimize the time or distance.

    %% Define variables to be used throughout the file
    global occupancy_matrix map_information...
        map_grid...
        points_grid gap_between_cells dx dy ...
        yx_2_idx_graph idx_graph_2_xy ...
        m_occupancy m_safe  ...
        node_location heap directions ...
        debug_mode file_path plan_debug ...
        map_velocity;
    %global path_points path_orientation
    
    plan_debug = false; % intermedium plots 
   
    % to be returned
    sampled_path = [];
    checkpoints = [];
    
    % a path need 2 points
    if(size(path_points,1)<2 && length(path_orientation)==2 )
        disp("2 points with 2D coordinates and orientation are needed ")
        return;
    end
  
    if(isempty(occupancy_matrix)); load(string(file_path+"occupancy_matrix.mat"), 'occupancy_matrix'); end
    if(isempty(map_information)); map_information=load(string(file_path+"mapInformation.mat")); end
    if(isempty(path_points)); load(string(file_path+"path_points.mat"),'path_points'); end
    
    safe_matrix = draw_safe_matrix;
    gap_between_cells = floor(1/map_information.meters_from_MAP);
    [points,allowed_points] = compute_map_grid(path_points);
    path_points = path_points(logical(allowed_points),:);

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
    m_occupancy = occupancy_matrix;
    m_occupancy(m_occupancy>4)=1;
    m_occupancy(m_occupancy==2)=1;
    m_occupancy(m_occupancy==3)=0.9;% slown down on traffic lights (the less, the lighter)
    m_occupancy(m_occupancy==4)=0;  % stop in Stop sings
    m_safe = 1-safe_matrix/max(max(safe_matrix));

    %% Auxiliar Structs
    heap = MinHeap(sum(map_grid,'all')/2);
    T = arrayfun(@(~) struct('cost',[],'distance',[],'index',[],'previous',[],'linear_velocity',[],...
                                'angular_velocity',[]), 1:size(map_grid,1)*size(map_grid,2), 'UniformOutput',false);
    node_location = horzcat(T{:});
    directions = struct('N',[0 -1],'NE',[1 -1],'E',[1 0],'SE',[1 1],'S',[0 1],'SW',[-1 1],'W',[-1 0],'NW',[-1 -1],...
                        'idxs' ,[0 -1;1 -1;1 0;1 1;0 1;-1 1;-1 0;-1 -1],...
                        'names', ["N","NE","E","SE","S","SW","W","NW"]);

    %% DYNAMIC DIJKSTRA - Path planning
    
    orientation = round_thetas(path_orientation); % initial and final orientation -- path_orientation

    itr=1;
    valid_points = 1:size(points_grid,1);
    n_max_points = length(valid_points);
        
    paths = cell(n_max_points-1,1);
    prevs = cell(n_max_points-1,1);
    
    wb=waitbar(0,"",'Name',"Planning The Best Route");
    wb.Position(1)= wb.Position(1)-wb.Position(3);
    tic
    while(itr<length(valid_points))
        
        start = points_grid(valid_points(itr),:);
        idx_start = yx_2_idx_graph(start(2),start(1));
        stop = points_grid(valid_points(itr+1),:);
        idx_stop = yx_2_idx_graph(stop(2),stop(1));
        
        % define the orientation for final point at the last path
        orientation_path = ["",""];  % auxilar vect of orentations (for each sub path)\
        % first point of the user chosen track
        if(valid_points(itr)==1)
            orientation_path(1) = orientation(1);
        end
        if(valid_points(itr+1)==n_max_points)
            orientation_path(2) = orientation(2);
            
            % force two end the path two points before in order to have
            removes_non_desired_neighbours(orientation(2),stop);
            % space to define the end orientation
            [~,I_front] = max(strcmp(directions.names,orientation(2)));
            % retrocede 1 movement
            step = directions.idxs(I_front,:);
            stop = stop-2*step;
            idx_stop = yx_2_idx_graph(stop(2),stop(1));
        end
        
        % performs dijkstra if the checkpoints are safe to drive
        if( safe_matrix(1+gap_between_cells*(start(2)-1),1+gap_between_cells*(start(1)-1))>0 && ...
            safe_matrix(1+gap_between_cells*(stop(2)-1),1+gap_between_cells*(stop(1)-1))>0 )
        
            node_location = horzcat(T{:});
            heap.Clear();
        
            wb=waitbar((itr-1)/(length(valid_points)-1),wb,"Route "+num2str(valid_points(itr))+" ->-> "+num2str(valid_points(itr+1)));
            dijkstra(idx_start,idx_stop,prevs{valid_points(itr)},loss_criterium,orientation_path);
        end
        
        if(isempty(node_location(idx_stop).cost))
            if(itr==1)  % failure in the initial sub path
                disp("Invalid Path (cannot start the path)");
                return
            elseif(itr==length(valid_points)-1)
                disp("Invalid Path (cannot end the path)");
                valid_points(itr+1)=[];
                break
            elseif(itr+2<=length(valid_points))
                disp("Invalid Subpath from point " + num2str(valid_points(itr)) + " to point " + num2str(valid_points(itr+1)));
                valid_points(itr:itr+1)=[];
                paths{valid_points(itr-1)} = [];
                itr=itr-1;
            else
                disp("Invalid Subpath from point " + num2str(valid_points(itr)) + " to point " + num2str(valid_points(itr+1)));
                break
            end
        else      
            paths{valid_points(itr)} = get_path(idx_start,idx_stop);
            prevs{valid_points(itr+1)} = node_location(idx_stop);
            disp("Valid Subpath from point " + num2str(valid_points(itr)) + " to point " + num2str(valid_points(itr+1)));
            itr=itr+1;
        end
    end
    
    toc
    delete(wb);
    
    path_data = [points_grid(1,:),zeros(1,4)];
    
    for subpath=valid_points
        if subpath == n_max_points; continue; end % last point
            path_data = [path_data;paths{subpath,:}];
    end
    
    if(isempty(path_data))
        disp("No Path available");
        return
    end

    if(n_max_points==valid_points(end))   % reach the last point
        path_data(end+1,:) = path_data(end,:);
        path_data(end,1:2) = points_grid(valid_points(end),:);
        path_data(end,5) = path_data(end-1,5)+2*sqrt(strlength(orientation(2)));
    end
    
    %% Path analysis
    n_points = length(path_data);
    path_distance = gap_between_cells*path_data(end,5)*map_information.meters_from_MAP;
    mean_velocity = map_velocity*sum(path_data(:,4))/n_points; 
    path_duration = 3.6*path_distance/mean_velocity;    % 1m/s = 3.6 Km/mh
    average_velocity = 3.6*gap_between_cells*norm(points_grid(1,:)-points_grid(end,:))*map_information.meters_from_MAP/path_duration;

    disp(" /'----------Path-Planning-----------'\")
    disp("| Distance:           "   +num2str(path_distance,"%.2f")   +  "  meters.  |")
    disp("| Duration:           "   +num2str(path_duration,"%.2f")   +  "   seconds. |")
    disp("| Mean Velocity:      "   +num2str(mean_velocity,"%.2f")   +  "   Km/h.    |")
    disp("| Speed:              "   +num2str(average_velocity,"%.2f")+  "   Km/h.    |")
    disp(" \,----------------------------------,/")
    
    %% transpose to the original map
    run_points = [(path_data(:,1)-1)*gap_between_cells+1 (path_data(:,2)-1)*gap_between_cells+1];
    save(string(file_path+"run_points.mat"), 'run_points');
    
    % there are not suficient number of points
    if(length(valid_points)<2);return;end
    

    %% validate checkpoints
    checkpoints=path_points(valid_points,:);
    save(string(file_path+"checkpoints.mat"), 'checkpoints');    
    
    %% Path Smoothing
    fig = uifigure;
    d=uiprogressdlg(fig,'Title','Smoothing the Path','Indeterminate','on');
    drawnow
    sampled_path = path_smoothing(run_points,checkpoints,map_information.meters_from_MAP);
    close(d);close(fig);
    
    
    %% Final plots and verifications
    if(debug_mode==true)
        inspect_plots(sampled_path, run_points, checkpoints, path_data, n_points, path_duration, map_velocity)
        disp("[EOF] Path Planning")
        %license('inuse')
        %[fList,pList] = matlab.codetools.requiredFilesAndProducts('path_planning.m');
    end

end

%% Visual Support Content
function inspect_plots(sampled_path, run_points, checkpoints, path_data, n_points, path_duration, map_velocity)
% This function displays in figures the path planned
    
    global map_information file_path safe_debug
    MAP = openfig(string(file_path+"MAP.fig"));
    MAP.Name = 'Path Planning Velocity';
    hold on
    plot(checkpoints(:,1),checkpoints(:,2),"wd","LineWidth",4)
    patch([run_points(:,1);NaN],[run_points(:,2);NaN],[path_data(:,4);NaN],...
        [map_velocity*path_data(:,4);NaN],'EdgeColor','interp',"Linewidth",2);
    cb=colorbar;
    cb.TickLabels=cb.TickLabels+" Km/h";
    cb.Position = [0.91 0.05 0.02 0.9];
    colormap(jet);
    Image = getframe(gcf);
    imwrite(Image.cdata, string(file_path+"dijkstra_path.png"), 'png');
    
    if(safe_debug==false); return; end
    
    f = figure('WindowStyle', 'docked');
    t=0:seconds(path_duration)/(n_points-1):seconds(path_duration)';

    subplot(2,1,1)
    hold on
    title("Run Velocity per distance")
    yyaxis left
    plot(path_data(:,5),path_data(:,4));
    ylabel("Linear Velocity [normalised]")
    yyaxis right
    plot(path_data(:,5),path_data(:,6));
    ylabel("Angular Velocity [rad]")
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
    lat = map_information.fget_Lat_from_MAP(sampled_path(:,2));
    lon = map_information.fget_Lon_from_MAP(sampled_path(:,1));
    geoplot(lat,lon,":",'Color',[0.4940 0.1840 0.5560],"LineWidth",2)
    hold on
    lat = map_information.fget_Lat_from_MAP(checkpoints(:,2));
    lon = map_information.fget_Lon_from_MAP(checkpoints(:,1));
    geoplot(lat,lon,"o",'Color',"#FFD700","MarkerSize",10,"LineWidth",3)
    

    %% Web map
    % webmap
    % wmline(lat,lon)
    % wmmarker(map_information.fget_Lat_from_MAP(smoothed_path(:,2)),map_information.fget_Lon_from_MAP(smoothed_path(:,1)))

end

%% Auxiliar functions to Path Planning
function dijkstra(idx_start,idx_finish,init_node,loss_criterium,orientation)
% Dynamical Weight Dijkstra where it is computed a path between two pixels,
% given priority two straight movement.
% It writes in the variable node_location the reachable points, and the
% algorithm stops either when the end point is reached or when there are
% not any good option to complete the path.

    global heap directions idx_graph_2_xy yx_2_idx_graph node_location
    
    in_heap = zeros(length(node_location),1);

    % Starting from the initial point
    if(isempty(init_node))
        heap.InsertKey(new_node(0,0,idx_start,"",0,0))
    else
        heap.InsertKey(init_node);
    end
    in_heap(idx_start) = 1;
    
    wb=waitbar(0,"",'Name',"Closiness of the destination");
    % Do Dijkstra
    while(heap.Count()>0)
        
        node = heap.ExtractMin();
        node_location(node.index)=node;
        in_heap(node.index) = -1;
        
        % Confirm if the end point was reached
        if(idx_finish==node.index)
            % final node
            delete(wb);
            return
        end
        waitbar(1-norm(idx_graph_2_xy(node.index)-idx_graph_2_xy(idx_finish))/...
            (norm(idx_graph_2_xy(idx_start)-idx_graph_2_xy(idx_finish))),...
            wb,...
            strjoin(repmat(".",1,1+mod(heap.Count(),10))));
        
        reachable_neighbours = identify_reachable_neighbours(idx_graph_2_xy(node.index),node.previous);

        for i = 1:8% arround movements
            if reachable_neighbours(i) == 0;continue;end
            
            step = directions.idxs(i,:);
            new_pos = idx_graph_2_xy(node.index)+step;
            
            % new node information
            new_idx = yx_2_idx_graph(new_pos(2),new_pos(1));
            direction=directions.names(i);
            [linear_velocity,angular_velocity] = compute_velocity(node.previous,directions.names(i),node.linear_velocity,node.angular_velocity,(directions.names(i)~=node.previous),new_pos);
            distance = node.distance + sqrt(strlength(directions.names(i)));
            cost = compute_cost(new_pos,linear_velocity,angular_velocity,distance,loss_criterium,node.cost);
            
            % Define the new atempt
            node_aux = new_node(cost,distance,new_idx,direction,linear_velocity,angular_velocity);
            
            % The first point has its speacific orientation
            if ((node.index==idx_start) && (strlength(orientation(1))>0) && (direction ~= orientation(1))); continue; end  
            
            %% Insert node at the heap
            
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
    subpath = zeros(0,6);
    
    % Could not reach the goal
    if(isempty(node_location(idx).cost))
        return
    end

    while(idx ~= idx_start)
        % add the last point
        subpath = [idx_graph_2_xy(idx),node_location(idx).cost,...
                   node_location(idx).linear_velocity,node_location(idx).distance,node_location(idx).angular_velocity;
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

function reachable_neighbours = identify_reachable_neighbours(xy,previous_direction)
% This function computes the points that are considered to be a next move.
% When it is the start point, the can move all over the place , however for
% any other circunstance, the car only of three choices: straight, left or
% rigth diagonal (degrees).

    global directions dx dy m_safe gap_between_cells points map_grid
   
   points = xy+directions.idxs;
   in_boundaries = logical(sum((points<=[dx dy]).*(points>=1),2)==2);
   points(~in_boundaries,:) = 1;
   
   % reachable points 
   reachable_neighbours = (1~=diag(m_safe(1+gap_between_cells*(points(:,2)-1),1+gap_between_cells*(points(:,1)-1))))... % inside map _grid
                            .*diag(map_grid(points(:,2),points(:,1)))...    % inside map _grid
                            .*in_boundaries;
   idxs = ones(1,8);
    if(previous_direction ~= "")
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
   
   % Availbale directions: directions.names(logical(reachable_neighbours))
end

function cost = compute_cost(end_pos,linear_velocity,angular_velocity,distance,loss_criterium,prev_cost)
% This function selects the loss function that is used to compute the
% cost of each node as well as its cost on the safetiness matrix
    
    global m_safe gap_between_cells
    
    if (loss_criterium=="velocity")
        % maximise linear velocity and minimise angular velocity
        loss = prev_cost+(1-linear_velocity)^2+(1+angular_velocity)^2;
    else % (loss_criterium=="distance")
        loss=distance;
    end
    
    %% This section takes into account also the cost of being in certain parts of the road, do not change
    cost = loss*(1+m_safe(1+gap_between_cells*(end_pos(2)-1),1+gap_between_cells*(end_pos(1)-1)));
end

function [linear_velocity,angular_velocity] = compute_velocity(start_dir,end_dir,prev_linear_velocity,prev_angular_velocity,change_of_direction,end_pos)
% This function simulates the velocity towards the time taking into
% consideration changes in direction

    global m_occupancy gap_between_cells map_information
    
    if (prev_linear_velocity<0.05)            % the car was stop in the last moment
        linear_velocity = 0.05;
    elseif (start_dir ~= end_dir)               % 45 degrees curve
    	linear_velocity = prev_linear_velocity*cos(pi/4);
    else                                        % straight movement
        velocity_increment = 1+map_information.meters_from_MAP*gap_between_cells/10;
        linear_velocity = min(1, prev_linear_velocity*velocity_increment);
    end

    % relieve the steering wheel after each change
    angular_velocity = prev_angular_velocity*0.5+(pi/4)*change_of_direction;
    
    % weights traffic light ans stop zones
    linear_velocity = linear_velocity*m_occupancy(1+gap_between_cells*(end_pos(2)-1),1+gap_between_cells*(end_pos(1)-1));
end

function thetas_names = round_thetas(thetas)
    global directions
    
    % avoid dimension errors
    thetas = reshape(thetas,2,1);
    
    % normalize to one circle
    thetas360 = wrapTo360(thetas);
    
    % round to the available dijkstra directions
    rounded_thetas = rem(round(thetas360/45),8)+1;
    
    % map between the nomation of the dijkstra and the coordantion
    % common knowledge
    map_coord = [3 2 1 8 7 6 5 4];
    
    thetas_dijkstra = rem(find(map_coord'==rounded_thetas'),8);
    thetas_dijkstra(thetas_dijkstra==0) = 8;
    
    thetas_names = directions.names(thetas_dijkstra);
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

function removes_non_desired_neighbours(orientation, point)
% This function will disallow the car to reach a neighour that will not
% imply the final orientation
    global directions map_grid 
    
    % final point
    for i=1:8
        [~,I_front] = max(strcmp(directions.names,orientation));
        if(-directions.idxs(I_front,:) == directions.idxs(i,:)); continue; end  % final orientation with oposite orientations
        
        % blocks with zero in the grid
        step = directions.idxs(i,:);
        neighbour = point+step;
        map_grid(neighbour(2),neighbour(1)) = 0;
    end
    
end

%% Safetiness Matrix
function safe_matrix = draw_safe_matrix(safe_distance, forbidden_zone)
% This function computes the safetiness matrix where it weights the
% neighbourhood and classifies whether is safe to drive in that zone or not

    global occupancy_matrix map_information debug_mode file_path plan_debug
    
    if nargin < 1
        safe_distance = 0.5;    % meters
        forbidden_zone = 1;  % meters
    end
    meters_from_MAP = map_information.meters_from_MAP;   % meters/pixel

    % ideal distance to preserve from obstacles
    safe_pixels = safe_distance/meters_from_MAP;
    forbidden_pixels = forbidden_zone/meters_from_MAP;

    % function speacifications
    inf_limit = 1;
    A = inf_limit/(log(safe_pixels+1));

    %   evaluates the potential risk from to the forbidden zone
    func_stable_region = @(x)A*log(x+1).*(x<=safe_pixels) + inf_limit*(x>=safe_pixels);
    
    %% draw the disk
    radius = ceil(forbidden_pixels);
    dist = 0:radius;
    v = [dist(1:end-1) flip(dist)];
    [X,Y] = meshgrid(v);
    disk = ((radius).^2) > (radius-X).^2+(radius-Y).^2;
    
    %   evaluates the danger zones
    Ch_disk = conv2(logical(occupancy_matrix), disk, 'same');
    safe_matrix_aux = round(Ch_disk .* logical(occupancy_matrix));
    safe_matrix_aux = safe_matrix_aux/max(max(safe_matrix_aux));
    safe_matrix_aux(safe_matrix_aux<1)=0;

    %% input vector to convulution
    radius = ceil(safe_pixels);
    v_up = 0:radius;
    v = [v_up(1:end-1) flip(v_up)];
    [X,Y] = meshgrid(v);
    u = func_stable_region(sqrt(2*(radius).^2) - sqrt((radius-X).^2+(radius-Y).^2));
    
    %   evaluates the safetiness of the road
    Ch = conv2(safe_matrix_aux, u, 'same');
    normalize = 255/max(max(Ch));
    safe_matrix = round(Ch*normalize .* safe_matrix_aux);
    save(string(file_path+"safe_matrix.mat"), 'safe_matrix');
    
    if((debug_mode && plan_debug)==false);return;end
    
    %% view
    
    f1=figure('WindowStyle', 'docked');
    mesh(disk,'EdgeColor', 'none','FaceColor', 'interp');
    light
    % lighting gouraud
    title("Forbidden Zone - Convulotion function");
    xlabel("pixels")
    ylabel("pixels")
    zlabel("weight")
    xlim([1 size(disk,1)])
    ylim([1 size(disk,2)])
    view(45,45)

    f2=figure('WindowStyle', 'docked');
    mesh(u,'EdgeColor', 'interp','FaceColor', 'interp');
    light
    % lighting gouraud
    title("Safetiness gradient Zone - Convolution function");
    xlabel("pixels")
    ylabel("pixels")
    zlabel("weight")
    xlim([1 size(u,1)])
    ylim([1 size(u,2)])
    view(45,45)

    f3=figure('WindowStyle', 'docked'); hold on
    merge_sf = [100*Ch_disk(round(1:end/3),:)/max(max(Ch_disk));...
                145*safe_matrix_aux(1+round(end/3):round(2*end/3),:);...
                safe_matrix(1+round(2*end/3):end,:)];
    mesh(merge_sf, 'EdgeColor', 'interp', 'FaceColor', 'flat');
    axis equal
    axis tight
    axis manual
    view(90,-90)
    title("Safetiness Area", "FontSize", 20, "FontName", "arial")
    xlabel("pixels")
    ylabel("pixels")
    colormap jet
    cb=colorbar;
    cb.Ticks = [0 255];
    cb.TickLabels=["Circuit Limitation","Safe Area"];
    cb.FontSize=14;
    cb.Location="South";
    cb.Position(2)=0.1;
end

%% Visibility Matrix
function [points,allowed_points] = compute_map_grid(path_points)
% It is computed a visibility matrix from the occupancy grid where the gap
% between the cells is the space of each division

    global occupancy_matrix gap_between_cells map_grid debug_mode file_path
    [dim_y, dim_x] = size(occupancy_matrix);

    dx = 1:gap_between_cells:dim_x;
    dy = 1:gap_between_cells:dim_y;
    [X,Y] = meshgrid(dx,dy);
    map_grid = occupancy_matrix(dy,dx)~=0;
    [points,allowed_points] = get_closest_point_in_grid(path_points);
    
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

function [points,allowed_points] = get_closest_point_in_grid(xy)
% Place the point selected by the user in the nearest point of the
% visibility matrix
    global file_path gap_between_cells occupancy_matrix
    
    points = [];    %   vector of neighbours in the grid
    allowed_points = zeros(size(xy,1),1);     %   real points inside the matrix
    %   float numbers inside the square
    rem_x = rem(xy(:,1),gap_between_cells);
    rem_y = rem(xy(:,2),gap_between_cells);
    
    %   The four corners of the closest square
    upper_left_corner = [xy(:,1)-rem_x+1, xy(:,2)-rem_y+1];
    upper_right_corner = [upper_left_corner(:,1)+gap_between_cells, upper_left_corner(:,2)];
    lower_left_corner = [upper_left_corner(:,1) upper_left_corner(:,2)+gap_between_cells];
    lower_right_corner = [upper_left_corner(:,1)+gap_between_cells upper_left_corner(:,2)+gap_between_cells];
    
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