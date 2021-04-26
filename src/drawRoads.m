function roadMarkers = drawRoads()    
    disp("Draw in the map one road (Start by picking 1 starting point and the ending point of the road: )");
    h = drawpolygon('Color','blue');
        
    % Change road color the mark it's done and retrieve pointer positions
    % that mark the road
    h.Color = [92 92 92] / 255;
    h.FaceAlpha = 0.6;
    roadMarkers = h.Position;
end

