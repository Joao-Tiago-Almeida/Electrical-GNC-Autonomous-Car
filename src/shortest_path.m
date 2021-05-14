clear all;
close all;
clc;

%% Load map information
global map_grid     % global filesin this file
load('../mat_files/occupancyMatrix.mat', 'occupancyMatrix');
load('../mat_files/safe_matrix.mat', 'safe_matrix');
load('../mat_files/pathPoints_grid.mat', 'points');
f_Graph = load('../mat_files/grid.mat');
map_grid = f_Graph.grid;

% convert the points in the occupancy matrix in the grid
global points_grid
points_grid(:,1) = (points(:,1)-1)/(f_Graph.x(1,2)-f_Graph.x(1,1))+1;
points_grid(:,2) = (points(:,2)-1)/(f_Graph.y(2,1)-f_Graph.y(1,1))+1;

% cell divisions
global d_occp_x d_occp_y dxy
dxy = size(map_grid,1);
d_occp_x = f_Graph.x(1,2)-f_Graph.x(1,1);
d_occp_y = f_Graph.y(2,1)-f_Graph.y(1,1);

% mapping between grid access and 1D vect idx
global yx_2_idx_graph idx_graph_2_xy
yx_2_idx_graph = @(y,x) dxy*(y-1)+x;
idx_graph_2_xy = @(idx) [rem(idx,dxy) (idx-rem(idx,dxy))/dxy+1];

% change to symmatric matrix since it is a minimization problem
global m_occupancy m_safe 
occupancyMatrix(occupancyMatrix==2)=1;  % ignoring crosswalks
occupancyMatrix(occupancyMatrix==3)=0.5;% slown down on traffic lights (the less, the lighter)
occupancyMatrix(occupancyMatrix==4)=0;  % stop in Stop sings
m_occupancy = 1-occupancyMatrix;
m_safe = 1-safe_matrix/max(max(safe_matrix));

%% Auxiliar Structs
global node_location heap directions
heap = MinHeap(sum(sum(map_grid))-1);
T = arrayfun(@(~) struct('cost',[],'index',[],'momentum',[],'previous',[]), 1:size(map_grid,1)*size(map_grid,2), 'UniformOutput',false);
node_location = horzcat(T{:});
directions = struct('N',[0 -1],'NE',[1 -1],'E',[1 0],'SE',[1 1],'S',[0 1],'SW',[-1 1],'W',[-1 0],'NW',[-1 -1],...
                    'idxs' ,[0 -1;1 -1;1 0;1 1;0 1;-1 1;-1 0;-1 -1],...
                    'names', ["N","NE","E","SE","S","SW","W","NW"]);

%% DYNAMIC DIJKSTRA - Path planning
%https://media.neliti.com/media/publications/165891-EN-shortest-path-with-dynamic-weight-implem.pdf

sub_path = [];  % last confirmed path but it migth not occured if the stop point has to be deleted 
path_and_cost = []; % accumulated path with start and stop points confirmed
prev_node=[];   % information about the start point 
prev_prev_node=[];  % information about the start point before
tic
i=1;
valid_points = 1:size(points_grid,1);
while(i<length(valid_points))
    start = points_grid(valid_points(i),:);
    idx_start = yx_2_idx_graph(start(2),start(1));
    stop = points_grid(valid_points(i+1),:);
    idx_stop = yx_2_idx_graph(stop(2),stop(1));
    
    dijkstra(idx_start,idx_stop,prev_node,"time");
    
    if(isempty(node_location(idx_stop).cost))
        valid_points(i)=[];
        i=i-1;
        prev_node = prev_prev_node;
        sub_path = [];
        disp("Invalid Subpath")
    else
        path_and_cost = [path_and_cost;sub_path];
        sub_path = get_path(idx_start,idx_stop);
        prev_prev_node = prev_node;
        prev_node = node_location(idx_stop);
        i=i+1;
    end
  
    node_location = horzcat(T{:});
    heap.Clear();
end
toc

% UNCOMENT when this moves to a function
% if(isempty(sub_path))
%     disp("No Path available");
%     return
% end

path_and_cost = [points_grid(1,:),0;    path_and_cost;  sub_path];

% transpose to the original map
run_points = [(path_and_cost(:,1)-1)*d_occp_x+1 (path_and_cost(:,2)-1)*d_occp_y+1];
save('../mat_files/run_points.mat', 'run_points');

%% Final plots and verifications
MAP = load('../mat_files/MAP.mat');
hold on
plot(points(:,1),points(:,2),"ko","LineWidth",5)
plot(run_points(:,1),run_points(:,2),"LineWidth",4)
Image = getframe(gcf);
imwrite(Image.cdata, '../mat_files/dijkstra_path.png', 'png');

figure('WindowStyle', 'docked');clf;
mesh(map_grid)
view(0,-90)
hold on
plot(points_grid(:,1),points_grid(:,2),"k*")
plot(path_and_cost(:,1),path_and_cost(:,2),"LineWidth",2)

figure('WindowStyle', 'docked');
hold on
plot(cumsum(path_and_cost(:,3)))
grid on
xlabel("Path")
ylabel("Accumulative cost")
title("Run Cost")
disp("EOF")
%% Auxiliar functions

function dijkstra(idx_start,idx_finish,init_node,loss_criterium)
    global heap directions idx_graph_2_xy yx_2_idx_graph node_location
    
    in_heap = zeros(length(node_location),1);

    % Starting from the initial point
    if(isempty(init_node))
        heap.InsertKey(new_node(0, idx_start,0,''))
    else
        heap.InsertKey(init_node);
    end
    in_heap(idx_start) = 1;
    
	reach_endpoint = false;

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
            disp("subpath with: "+num2str(sum(in_heap==-1))+" points.")
            return
        end
        
        reachable_neighbours = identify_reachable_neighbours(idx_graph_2_xy(node.index),node.previous, sum(in_heap==-1)==1);

        for i = 1:8% arround movements
            if reachable_neighbours(i) == 0;continue;end
            
            step = directions.idxs(i,:);
            new_pos = idx_graph_2_xy(node.index)+step;
            
            % new node informatino
            new_idx = yx_2_idx_graph(new_pos(2),new_pos(1));
            direction=directions.names(i);
            momentum = compute_momentum(node.previous,directions.names(i),node.momentum,new_pos);
            cost = compute_cost(new_pos,momentum,node.cost,directions.names(i)==node.previous,loss_criterium);
            
%             if(new_idx==15465)
%                 disp("o que ta a acontecer?")
%             end
            
            % Define the new atempt
            node_aux = new_node(cost,new_idx,momentum,direction);
                
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
     
end

function subpath = get_path(idx_start,idx_finish)
    global node_location directions yx_2_idx_graph idx_graph_2_xy
    idx = idx_finish;
    subpath = zeros(0,3);
    
    % Could not reach the goal
    if(isempty(node_location(idx).cost))
        return
    end
    
    while(idx ~= idx_start)
        % add the last point
        subpath = [idx_graph_2_xy(idx),node_location(idx).cost;subpath];
        
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
   global directions map_grid dxy
   
   points = xy+directions.idxs;
   in_boundaries = logical(sum((points<=dxy).*(points>=1),2)==2);
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

function cost = compute_cost(end_pos,momentum,cost_previous,straight,loss_criterium)
    global m_safe d_occp_x d_occp_y
    
    if (loss_criterium=="time")
        loss = (1-momentum);
    else % (loss_criterium=="distance")
        loss=cost_previous+straight+(~straight)*sqrt(2);
    end
    
    %% This section represents the cost of being in certain parts of the road, do not change
    cost = loss*(1+m_safe(1+d_occp_y*(end_pos(2)-1),1+d_occp_x*(end_pos(1)-1)));
end

function momentum = compute_momentum(start_dir,end_dir,prev_momentum,end_pos)
    global m_occupancy d_occp_x d_occp_y
    
    if (prev_momentum==0)    % the car was stop in the last moment
        momentum = 0.08;
    elseif (start_dir ~= end_dir) % 45degrees curve
        momentum = prev_momentum*atan(d_occp_y/d_occp_x);
    else    % straight movement
        momentum_increment = 1.09;
        momentum = min(1,prev_momentum*momentum_increment);
    end
    
    % weights traffic light ans stop zones
    momentum = momentum*(1-m_occupancy(1+d_occp_y*(end_pos(2)-1),1+d_occp_x*(end_pos(1)-1)));
end

function s = new_node(c, i, m, p)

    % cost to reach the node
    f_cost = 'cost';
    v_cost = c;
    
    % index in the graph
    f_index = 'index';
    v_index = i;
    
    % momentum
    f_momentum = 'momentum';
    v_momentum = m;
    
    % previous node
    f_previous = 'previous';
    v_previous = p;
    
    s = struct(f_cost,v_cost,f_index,v_index,f_momentum,v_momentum,f_previous,v_previous);
end

