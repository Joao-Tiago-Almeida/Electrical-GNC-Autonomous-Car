function [point, dist, theta, k_p] = dist_to_traj(posx, posy, trajx, trajy, trajt, v, stp, k_p)
    global fixed_sample_rate max_velocity
    point = [0,0];
    dist = Inf;
    theta = 0;
    %jump to a future iteration
    jump = ceil(1/stp);
    %if the car starts at k_p knowing his max velocity it can oly reach
    %k_p+tpp
    tpp = ceil((max_velocity*0.1*2)/fixed_sample_rate);
    for k = k_p:min(k_p+tpp,length(trajx))
        %find the closest point in the path
        distt = norm([posx-trajx(k),posy-trajy(k)]);
        if distt < dist
            dist = distt;
            k_p = k;
            passe = k+ceil(jump);
            if passe > length(trajx)
                passe = length(trajx);
            end
            %save the point that should be followed
            point = [trajx(passe),trajy(passe)];
            theta = trajt(passe);
        end
    end
end