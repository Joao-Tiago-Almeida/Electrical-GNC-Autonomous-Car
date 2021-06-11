function thetat = theta_generator(xt,yt)
    %generates theta along the path
    thetat = zeros(length(xt),1);
    %theta is given by the arctan(dy/dx)
    for k = 1:length(xt)-1
        xa = xt(k+1)-xt(k);
        ya = yt(k+1)-yt(k);
        thetat(k) = atan2(ya,xa);
    end
    thetat(k+1) = thetat(k);
end