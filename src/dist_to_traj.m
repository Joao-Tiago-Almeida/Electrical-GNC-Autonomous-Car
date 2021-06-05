function [point, dist, theta, k_p] = dist_to_traj(posx, posy, trajx, trajy, trajt, v, stp, k_p)
    point = [0,0];
    dist = Inf;
    theta = 0;
    jump = ceil(v/stp);
    for k = k_p:length(trajx)
        distt = norm([posx-trajx(k),posy-trajy(k)]);
        if distt < dist
            dist = distt;
            k_p = k;
            passe = k+ceil(jump);%6 para o testing robot
            if passe > length(trajx)
                passe = length(trajx);
            end
            point = [trajx(passe),trajy(passe)];
            theta = trajt(passe);
        end
    end
end