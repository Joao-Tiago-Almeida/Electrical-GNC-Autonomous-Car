function [connectivityMatrix, nodesPosition] = create_graph(Polygon)

if nargin < 1
    load('polyin.mat', 'h');
end

T = triangulation(Polygon);

%T_new = subdivide_triangles(T, 1)

figure; hold on;
triplot(T);
[connectivityMatrix, nodesPosition] = build_connectivity_matrix(T);
%plot(IC(:,1), IC(:,2), 'r*');
%viscircles(IC, r);
axis equal

[IC,r] = incenter(T);


% axis tight
% axis manual
end
function [connectivityMatrix, nodesPosition] = build_connectivity_matrix(T)
    connectivityMatrix = zeros(length(T.ConnectivityList(:,1)));
    nodesPosition = zeros(length(T.ConnectivityList(:,1)), 2);
    %Compute the nodes positions
    for i=1:length(T.ConnectivityList(:,1))
        nodesPosition (i, 1) = (T.Points(T.ConnectivityList(i,1), 1) + T.Points(T.ConnectivityList(i,2), 1) + T.Points(T.ConnectivityList(i,3), 1))/3;
        nodesPosition (i, 2) = (T.Points(T.ConnectivityList(i,1), 2) + T.Points(T.ConnectivityList(i,2), 2) + T.Points(T.ConnectivityList(i,3), 2))/3;  
        plot(nodesPosition(i, 1), nodesPosition(i,2), 'ro');
    end
    %DESCULPA ALMEIDA
    tic
    %[X, Y] = meshgrid(length(T.ConnectivityList(:,1)), length(T.ConnectivityList(:,1)));
    for i = 1:length(T.ConnectivityList(:,1))
        for j = (i+1):length(T.ConnectivityList(:,1))
            if(sum(ismember(T.ConnectivityList(i, :),T.ConnectivityList(j, :))) == 2)
                connectivityMatrix(i, j) = 1;
                connectivityMatrix(j, i) = 1;
                %plot connectivity lines
                plot([nodesPosition(i, 1), nodesPosition(j, 1)], [nodesPosition(i, 2), nodesPosition(j, 2)], 'k');
            end
        end
    end
    toc
end

function T_new = subdivide_triangles(T, N)

for j = 1:N
    subPoints =[];
    subConnectivityList =[];
    for i=1:length(T.ConnectivityList)

        triangle_points = T.Points(T.ConnectivityList(i,:), :);
        sub_Triangle = triangulation(polyshape(triangle_points(:,1), triangle_points(:,2)));

        sub_center = incenter(sub_Triangle);

        sub_triangle_1 = @(X0) polyarea( [triangle_points([1,2], 1); X0(1)],  [triangle_points([1,2], 2); X0(2)]);
        sub_triangle_2 = @(X0) polyarea( [triangle_points([1,3], 1); X0(1)],  [triangle_points([1,3], 2); X0(2)]);
        sub_triangle_3 = @(X0) polyarea( [triangle_points([2,3], 1); X0(1)],  [triangle_points([2,3], 2); X0(2)]);
        minimize_3_areas = @(X0) 1e10*(~inpolygon(X0(1), X0(2), triangle_points(:,1), triangle_points(:,2))) + (sub_triangle_1(X0)-sub_triangle_2(X0))^2 + (sub_triangle_1(X0)-sub_triangle_3(X0))^2 + (sub_triangle_2(X0)-sub_triangle_3(X0))^2;

        aux2 = fminsearch(minimize_3_areas, sub_center);
        
        if (inpolygon(aux2(1), aux2(2), triangle_points(:,1), triangle_points(:,2)) == 0)
            aux2 = sub_center;
        end
       

        new_triangle_points = delaunayTriangulation([triangle_points(:,1);aux2(1)], [triangle_points(:,2);aux2(2)]);

        subPoints = [subPoints; new_triangle_points.Points];
        subConnectivityList = [subConnectivityList; new_triangle_points.ConnectivityList+4*(i-1)];
    end

    T_new = triangulation(subConnectivityList, subPoints);
    T=T_new;
end

end