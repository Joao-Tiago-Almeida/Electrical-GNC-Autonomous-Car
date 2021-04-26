function crosswalk = drawCrosswalk()
    disp("Draw in the map one crosswalk by drawing the region");
    h = drawpolygon('Color','blue');
        
    h.Color = 'white';
    h.FaceAlpha = 0.6;
    crosswalk = h.Position;
end

