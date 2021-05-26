function thetat = theta_generator(xt,yt)
    thetat = zeros(length(xt),1);
    for k = 1:length(xt)-1
        xa = xt(k+1)-xt(k);
        ya = yt(k+1)-yt(k);
        thetat(k) = atan2(ya,xa);
    end
    thetat(k+1) = thetat(k);
end