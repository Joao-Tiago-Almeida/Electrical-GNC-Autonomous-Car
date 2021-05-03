function crosswalk = drawTrafficLight()
    disp("Draw in the map one traffic light by drawing the region");
    h = drawpolygon('Color','blue');
        
    h.Color = 'green';
    h.FaceAlpha = 0.6;
    h.Label = 'Traffic Light';
    crosswalk = h.Position;
end

