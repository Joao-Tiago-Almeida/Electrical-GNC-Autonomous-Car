clear all;
close all;
%clc;

%% Load map information
global map_grid points MAP_info     % global vars in this file
load('../mat_files/occupancyMatrix.mat', 'occupancyMatrix');
load('../mat_files/safe_matrix.mat', 'safe_matrix');
load('../mat_files/pathPoints_grid.mat', 'points');
MAP_info = load('../mat_files/mapInformation.mat');
f_Graph = load('../mat_files/grid.mat');
map_grid = f_Graph.grid;

% convert the points in the occupancy matrix in the grid
global points_grid
points_grid(:,1) = (points(:,1)-1)/(f_Graph.x(1,2)-f_Graph.x(1,1))+1;
points_grid(:,2) = (points(:,2)-1)/(f_Graph.y(2,1)-f_Graph.y(1,1))+1;

% cell divisions
global d_occp_xy dx dy
dx = size(map_grid,2);
dy = size(map_grid,1);
d_occp_xy = f_Graph.x(1,2)-f_Graph.x(1,1);

% mapping between grid access and 1D vect idx
global yx_2_idx_graph idx_graph_2_xy
yx_2_idx_graph = @(y,x) dx*(y-1)+x;
idx_graph_2_xy = @(idx) [rem(idx,dx) (idx-rem(idx,dx))/dx+1];

% change to symmatric matrix since it is a minimization problem
global m_occupancy m_safe 
occupancyMatrix(occupancyMatrix==2)=1;  % ignoring crosswalks
occupancyMatrix(occupancyMatrix==3)=1;% slown down on traffic lights (the less, the lighter)
occupancyMatrix(occupancyMatrix==4)=1;  % stop in Stop sings
m_occupancy = 1-occupancyMatrix;
m_safe = 1-safe_matrix/max(max(safe_matrix));

%% Auxiliar Structs
global node_location heap directions
heap = MinHeap(sum(sum(map_grid))-1);
T = arrayfun(@(~) struct('cost',[],'distance',[],'index',[],'previous',[],'velocity',[]), 1:size(map_grid,1)*size(map_grid,2), 'UniformOutput',false);
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

% UNCOMENT when this moves to a function
if(isempty(sub_path))
    disp("No Path available");
    return
end

path_data = [points_grid(1,:),zeros(1,size(path_data,2)-2);    path_data;  sub_path];

% Path analysis
max_velocity=30; %Km/h
n_points = length(path_data);
path_distance = d_occp_xy*path_data(end,5)*MAP_info.meters_from_MAP;
mean_velocity = max_velocity*sum(path_data(:,4))/n_points; 
path_duration = 3.6*path_distance/mean_velocity;    % 1m/s = 3.6 Km/mh
average_velocity = 3.6*d_occp_xy*norm(points_grid(1,:)-points_grid(end,:))*MAP_info.meters_from_MAP/path_duration;
disp(" /'---------Path-Information---------'\")
disp("| Points:             "   +num2str(n_points)             +  "     pixels.  |")
disp("| Distance:           "   +num2str(path_distance,"%.2f")   +  "  meters.  |")
disp("| Duration:           "   +num2str(path_duration,"%.2f")   +  "   seconds. |")
disp("| Mean Velocity:      "   +num2str(mean_velocity,"%.2f")   +  "   Km/h.    |")
disp("| Average Velocity:   "   +num2str(average_velocity,"%.2f")+  "   Km/h.    |")
disp(" \,----------------------------------,/")
% transpose to the original map
run_points = [(path_data(:,1)-1)*d_occp_xy+1 (path_data(:,2)-1)*d_occp_xy+1];
save('../mat_files/run_points.mat', 'run_points');

% validate checkpoints
checkpoints=points(valid_points,:);
save('../mat_files/checkpoints.mat', 'checkpoints');

%% Final plots and verifications
inspect_plots(run_points, path_data, n_points, path_duration, max_velocity)
disp("EOF")

path_smoothing

function inspect_plots(run_points, path_data, n_points, path_duration, max_velocity)
    global points MAP_info
    MAP = load('../mat_files/MAP.mat');
    hold on
    plot(points(:,1),points(:,2),"ko","LineWidth",6)
    %plot(run_points(:,1),run_points(:,2),"LineWidth",1)
    %place_car(run_points,10)
    p = patch([run_points(:,1);NaN],[run_points(:,2);NaN],[path_data(:,4);NaN],[max_velocity*path_data(:,4);NaN],'EdgeColor','interp',"Linewidth",5);
    cb=colorbar;
    cb.TickLabels=cb.TickLabels+" Km/h";
    cb.Position = [0.91 0.05 0.02 0.9];
    colormap(jet);
    Image = getframe(gcf);
    imwrite(Image.cdata, '../mat_files/dijkstra_path.png', 'png');

    % figure('WindowStyle', 'docked');clf;
    % mesh(map_grid)
    % view(0,-90)
    % hold on
    % plot(points_grid(:,1),points_grid(:,2),"k*")
    % patch(path_data(:,1),path_data(:,2),path_data(:,4),path_data(:,4),'EdgeColor','interp');

    % figure('WindowStyle', 'docked');
    % hold on
    % plot(path_data(:,3))
    % grid on
    % xlabel("Path")
    % ylabel("Accumulative cost")
    % title("Run Cost")

    figure('WindowStyle', 'docked');
    t=0:seconds(path_duration)/(n_points-1):seconds(path_duration)';
    tiledlayout(3,1);
    % 
    % % Tile 1
    nexttile
    hold on
    title("Run Velocity per time")
    plot(t,path_data(:,4));
    xlim([0 seconds(path_duration)])
    % 
    % % Tile 2
    nexttile
    hold on
    title("Run Velocity per distance")
    plot(path_data(:,5),path_data(:,4));
    xlim([0 path_data(end,5)])
    % 
    % % Tile 3
    nexttile
    hold on
    title("Run Cost")
    plot(t,path_data(:,3));
    xlim([0 seconds(path_duration)])

    figure('WindowStyle', 'docked');
    lat = MAP_info.fget_Lat_from_MAP(run_points(:,2));
    lon = MAP_info.fget_Lon_from_MAP(run_points(:,1));
    geoplot(lat,lon,'g-*')

    %% Web map
    % webmap
    % wmline(lat,lon)
    % wmmarker(MAP_info.fget_Lat_from_MAP(points(:,2)),MAP_info.fget_Lon_from_MAP(points(:,1)))

end
%% Auxiliar functions

function dijkstra(idx_start,idx_finish,init_node,loss_criterium)
    global heap directions idx_graph_2_xy yx_2_idx_graph node_location
    
    in_heap = zeros(length(node_location),1);

    % Starting from the initial point
    if(isempty(init_node))
        heap.InsertKey(new_node(0,0,idx_start,'',0))
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
        
        if(1200==node.index)
            disp("Hello World")
        end
        
        % Confirm if the end point was reached
        if(idx_finish==node.index)
            disp("Computed points: "+num2str(sum(in_heap==-1)))
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
            velocity = compute_velocity(node.previous,directions.names(i),node.velocity,new_pos);
            distance = node.distance+(directions.names(i)==node.previous)+(directions.names(i)~=node.previous)*sqrt(2);            
            cost = compute_cost(new_pos,velocity,distance,loss_criterium,node.cost);
            
            % Define the new atempt
            node_aux = new_node(cost,distance,new_idx,direction,velocity);
                
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
                   node_location(idx).velocity,node_location(idx).distance;
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
   global directions map_grid dx dy
   
   points = xy+directions.idxs;
   in_boundaries = logical(sum((points<=[dx dy]).*(points>=1),2)==2);
   points(~in_boundaries,:) = 1;
   
   reachable_neighbours = diag(map_grid(points(:,2),points(:,1))).*in_boundaries;  % at this point it contains all 
   idxs = ones(1,8);
    if(previous_direction ~= '')
        % When the previous direcition is one of {N;E;S;W}, the next
        % direction has to contain this letter in the direction name
        % once the available movements are withing [-90,90]degrees.
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

function cost = compute_cost(end_pos,velocity,distance,loss_criterium,prev_cost)
    global m_safe d_occp_xy
    
    if (loss_criterium=="time")
        loss = prev_cost+(1-velocity);
    else % (loss_criterium=="distance")
        loss=distance;
    end
    
    %% This section represents the cost of being in certain parts of the road, do not change
    cost = loss*(1+m_safe(1+d_occp_xy*(end_pos(2)-1),1+d_occp_xy*(end_pos(1)-1)));
end

function velocity = compute_velocity(start_dir,end_dir,prev_velocity,end_pos)
    global m_occupancy d_occp_xy
    
    if (prev_velocity==0)    % the car was stop in the last moment
        velocity = 0.05;
    elseif (start_dir ~= end_dir) % 45 degrees curve
    	velocity = prev_velocity*atan(1);
    else                        % straight movement
        velocity_increment = 1.1;
        velocity = min(1,prev_velocity*velocity_increment);
    end
    
    % weights traffic light ans stop zones
    velocity = velocity*(1-m_occupancy(1+d_occp_xy*(end_pos(2)-1),1+d_occp_xy*(end_pos(1)-1)));
end

function s = new_node(c, d, i, p, v)

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
    
    % velocity
    f_velocity = 'velocity';
    v_velocity = v;
   
    s = struct(f_cost,v_cost,f_distance,v_distance,f_index,v_index,f_previous,v_previous,f_velocity,v_velocity);
end

