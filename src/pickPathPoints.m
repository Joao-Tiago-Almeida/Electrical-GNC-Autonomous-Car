function pathPoints = pickPathPoints(occupancyMatrix)

if nargin < 1
    load('../mat_files/occupancyMatrix.mat', 'occupancyMatrix');
end

disp("Pick in the map the INITIAL point, the intermediate points and the FINAL point");

imshow(occupancyMatrix);

pathPoints = [];
point_not_valid = true;
while(point_not_valid || size(pathPoints, 1) < 2)
    try
        h1 = drawcrosshair('LineWidth',1, 'Color','k');
    catch
        h1 = drawpoint('LineWidth',1, 'Color','k');
    end
    
    try
        h1.Visible = 'off';
    catch
        h1.Visibility = 'off';
    end
    if ( occupancyMatrix( round(h1.Position(2)), round(h1.Position(1)) ) == 0)
        disp("Please pick a point inside a road!!!");
        continue;
    end
    
    hc = drawcircle('Center',h1.Position,'Radius',14,'StripeColor','cyan');
    hc.InteractionsAllowed = 'none';
    
    pathPoints = [pathPoints; round(h1.Position)];

    valid = input("If you want to keep adding points press enter. If not press 'q'. ", 's');
    if(strcmp(valid, "q")==1)
        if(size(pathPoints, 1) < 2)
            disp("Insert at least two points!")
            continue;
        end
        point_not_valid = false;
    end

    h.Color = 'magenta';
    h.FaceAlpha = 0.8;
end

save('../mat_files/pathPoints.mat', 'pathPoints');

Image = getframe(gcf);
imwrite(Image.cdata, '../mat_files/MAP_w_roads.png', 'png');

end


