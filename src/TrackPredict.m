function theta_var_10m = TrackPredict(thetat, dist, k_p)
    n = 10/dist;%dist in meters
    theta_var_10m = 0;
    for i = k_p:k_p+n
        if i > length(thetat)
            break;
        end
        theta_var_10m = theta_var_10m + thetat(i);
    end
    theta_var_10m = theta_var_10m/(i-k_p);
    theta_var_10m = theta_var_10m - thetat(k_p);
end