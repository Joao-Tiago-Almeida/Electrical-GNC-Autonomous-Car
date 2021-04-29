function create_graph(Polygon, binnary_matrix)

polygon.x = Polygon.Vertices(:,1)';      %   Polygon x-coordinates
polygon.y = Polygon.Vertices(:,2)';      %   Polygon y-coordinates

NX = 100;
NY = 100;

PXY = DIVIDEXY(polygon,NX,NY);

subplot(1,2,1);   %Plot original Polygon
for i=0:1:NX
    plot([i/NX*(max(polygon.x)-min(polygon.x))+min(polygon.x) i/NX*(max(polygon.x)-min(polygon.x))+min(polygon.x)],[min(polygon.y) max(polygon.y)],'g');
    hold on
end
for i=0:1:NY
    plot([min(polygon.x) max(polygon.x)],[i/NY*(max(polygon.y)-min(polygon.y))+min(polygon.y) i/NY*(max(polygon.y)-min(polygon.y))+min(polygon.y)],'g');
    hold on
end
plot([polygon.x ],[polygon.y ],'b*-');
hold off
daspect([1 1 1]);
subplot(1,2,2);   %Plot smaller polygons set by grid
for i=1:1:NX
for j=1:1:NY
    if not(isempty(PXY{i,j})) && sum(diag(binnary_matrix(round(PXY{i,j}.x), round(PXY{i,j}.y)))) ==  length(PXY{i,j}.x)
    plot([PXY{i,j}.x PXY{i,j}.x(1)],[PXY{i,j}.y PXY{i,j}.y(1)],'ro-');
    end
hold on
end
end
hold off
daspect([1 1 1]);

disp("Gamei uma função ");
end

function grid = build_grid(polygon,nx,ny)

% computes the division
dx=(max(polygon.x)-min(polygon.x))/nx;
dy=(max(polygon.y)-min(polygon.y))/ny;



end

function PXY=DIVIDEXY(polygon,NX,NY)
    %Input
    %polygon: a structure consist of polygon.x (vector of x-coordinates) and polygon.y (vector of y-coordinates) 
    %NX: Number of divisions in x direction
    %NY: Number of divisions in y direction
    %Output
    %PXY: a cell array where PX{i,j}.x and PX{i,j} are vectors of x and y coordinates of new polygon in (i,j) grid position
    DX=(max(polygon.x)-min(polygon.x))/NX;
    DY=(max(polygon.y)-min(polygon.y))/NY;
    i=0;
    P=polygon;
    for X=min(polygon.x)+DX:DX:max(polygon.x)-DX
        i=i+1;
        [PX{i}, P]=DIVIDEX(P,X);

    end
        PX{NX}=P;
    for i=1:1:NX
        j=0;
        for Y=min(polygon.y)+DY:DY:max(polygon.y)-DY
            j=j+1;
            [PXY{i,j}, PX{i}]=DIVIDEY(PX{i},Y);     
        end
        PXY{i,NY}=PX{i};
    end
end

function [polygon1 polygon2]=DIVIDEX(polygon,X)
polygon1=[];
polygon2=[];
if not(isempty(polygon))
    m=length(polygon.x);
    c1=0;
    c2=0;
for i=1:1:m
j=i+1;
if i==m
j=1;
end 
if polygon.x(i)<=X
c1=c1+1;
polygon1.x(c1)=polygon.x(i);
polygon1.y(c1)=polygon.y(i);
end
if polygon.x(i)>=X
c2=c2+1;
polygon2.x(c2)=polygon.x(i);
polygon2.y(c2)=polygon.y(i);  
end
if (polygon.x(i)>X && polygon.x(j)<X) || (polygon.x(i)<X && polygon.x(j)>X) 
c1=c1+1;
polygon1.x(c1)=X;
polygon1.y(c1)=polygon.y(j)+(polygon.y(i)-polygon.y(j))/(polygon.x(i)-polygon.x(j))*(X-polygon.x(j));    
c2=c2+1;
polygon2.x(c2)=X;
polygon2.y(c2)=polygon.y(j)+(polygon.y(i)-polygon.y(j))/(polygon.x(i)-polygon.x(j))*(X-polygon.x(j));           
end
    
  
    
end
end
end

function [polygon1 polygon2]=DIVIDEY(polygon,Y)
polygon1=[];
polygon2=[];
if not(isempty(polygon))
m=length(polygon.y);
c1=0;
c2=0;
for i=1:1:m
j=i+1;
if i==m
j=1;
end 
if polygon.y(i)<=Y
c1=c1+1;
polygon1.x(c1)=polygon.x(i);
polygon1.y(c1)=polygon.y(i);
end
if polygon.y(i)>=Y
c2=c2+1;
polygon2.x(c2)=polygon.x(i);
polygon2.y(c2)=polygon.y(i);  
end
if (polygon.y(i)>Y && polygon.y(j)<Y) || (polygon.y(i)<Y && polygon.y(j)>Y) 
c1=c1+1;
polygon1.x(c1)=polygon.x(j)+(polygon.x(i)-polygon.x(j))/(polygon.y(i)-polygon.y(j))*(Y-polygon.y(j));    
polygon1.y(c1)=Y;
c2=c2+1;
polygon2.x(c2)=polygon.x(j)+(polygon.x(i)-polygon.x(j))/(polygon.y(i)-polygon.y(j))*(Y-polygon.y(j));           
polygon2.y(c2)=Y;
end
    
  
    
end
end
end