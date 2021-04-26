load('polyin.mat', 'h');

%pgon = polyshape(h.Position(:,1),h.Position(:,2));
%plot(pgon);

Positions = [zeros(size(h.Position)); zeros(size(h.Position))];
for i =1:length(h.Position)-1
    Positions(((i-1)*2)+1, 1) = h.Position(i, 1);
    Positions(((i-1)*2)+1, 2) = h.Position(i, 2);
    Positions(((i-1)*2)+2, 1) =  (h.Position(i, 1) + h.Position(i+1, 1)) / 2;
    Positions(((i-1)*2)+2, 2) = (h.Position(i, 2) + h.Position(i+1, 2)) / 2;
end

T = delaunayTriangulation(h.Position(:,1),h.Position(:,2));
figure();
triplot(T)