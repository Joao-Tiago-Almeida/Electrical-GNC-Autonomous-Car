function [stp, E] = Energy_decreasing(v, vant, P0, E)
    M = 810;
    deltaE = M*v*(v-vant)+P0*0.1;
    E = E-deltaE;
    if E <= 0
        stp = true;
    else
        stp = false;
    end
end
