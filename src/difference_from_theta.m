function t_diff = difference_from_theta(theta_a, theta_b)
    %the difference between two angles should take into account the unit
    %circle
    dif = norm(theta_a-theta_b);
    dif2 = theta_a-theta_b;
    %if the difference is bigger than pi, the shortest way to reduce this
    %difference is the symetric path
    if dif > pi
        dif2 = -dif2;
    end
    %the actual difference has the sign of the absolute difference and is
    %given by that difference minus pi, if the difference is bigger than pi
    t_diff = sign(dif2) * (dif - sgn(dif-pi)*(dif-pi)*2);

end

%sgn function based on sign but instead of being -1 when x<0, is 0
function sg = sgn(x)
    if x >= 0
        sg = 1;
    else
        sg = 0;
    end
end