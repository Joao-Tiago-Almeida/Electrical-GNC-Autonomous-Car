function pathPoints = pickPathPoints(binaryMatrix)
    disp("Pick in the map the INITIAL point, the intermediate points and the FINAL point");
    pathPoints = [];
    point_not_valid = true;
    while(point_not_valid && size(pathPoints, 2) > 1)
        try
            h1 = drawcrosshair('LineWidth',1, 'Color','k');
        catch
            h1 = drawpoint('LineWidth',1, 'Color','k');
        end
        
        if ( binaryMatrix( round(h1.Position(2)), round(h1.Position(1)) ) == 0)
            disp("Please pick a point inside a road!!!");
            h1.Visibility = 'off';
            continue;
        end
        
        pathPoints = [pathPoints h1];
        
        valid = input("If you want to keep adding points press enter. If not press 'q'. ", 's');
        if(strcmp(valid, "q")==1)
            point_not_valid = false;
        end

        h.Color = 'magenta';
        h.FaceAlpha = 0.8;
    end
end


