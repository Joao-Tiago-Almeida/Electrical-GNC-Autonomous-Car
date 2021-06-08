function [point, dist, theta, k_p] = dist_to_traj(posx, posy, trajx, trajy, trajt, v, stp, k_p)
    global fixed_sample_rate max_velocity
    point = [0,0];
    dist = Inf;
    theta = 0;
    jump = ceil(v/stp);
    tpp = ceil((max_velocity*0.1*2)/fixed_sample_rate);
    for k = k_p:min(k_p+tpp,length(trajx))
        distt = norm([posx-trajx(k),posy-trajy(k)]);
        if distt < dist
            dist = distt;
            k_p = k;
            passe = k+ceil(jump);
            if passe > length(trajx)
                passe = length(trajx);
            end
            point = [trajx(passe),trajy(passe)];
            theta = trajt(passe);
        end
    end
end