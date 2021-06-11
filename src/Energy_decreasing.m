function [stp, E] = Energy_decreasing(v, vant, P0, E)
    M = 810;
    %formula given in the guide
    deltaE = M*v*(v-vant)+P0*0.1;
    E = E-deltaE;
    %check if battery is empty
    if E <= 0
        stp = true;
    else
        stp = false;
    end
end
