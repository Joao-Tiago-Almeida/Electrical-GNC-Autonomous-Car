function stopSign = drawStopSign()
    disp("Draw in the map one stop sign by drawing the region");
    h = drawpolygon('Color','blue');
        
    h.Color = 'red';
    h.FaceAlpha = 0.6;
    stopSign = h.Position;
end
