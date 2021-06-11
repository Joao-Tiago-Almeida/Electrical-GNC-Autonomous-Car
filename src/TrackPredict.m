function theta_var_10m = TrackPredict(thetat, dist, k_p)
    n = 10/dist;%dist in meters
    theta_var_10m = 0;
    %sum of all the angles in 10 meters
    for i = k_p:k_p+n
        if i > length(thetat)
            break;
        end
        theta_var_10m = theta_var_10m + thetat(i);
    end
    %mean angle for the next 10 meters
    theta_var_10m = theta_var_10m/(i-k_p);
    %the mean difference is the difference between the mean theta and the
    %value of theta at iteration k_p
    theta_var_10m = theta_var_10m - thetat(k_p);
end