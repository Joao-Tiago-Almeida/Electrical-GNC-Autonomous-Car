function roadMarkers = drawRoads()    
    disp("Draw in the map one road");
    h = drawpolygon('Color','blue');
        
    % Change road color the mark it's done and retrieve pointer positions
    % that mark the road
    h.Color = [130 130 130] / 255;
    h.FaceAlpha = 0.6;
    h.Label = 'Road';
    roadMarkers = h.Position;
end

