function t_diff = difference_from_theta(theta_a, theta_b)
    dif = norm(theta_a-theta_b);
    dif2 = theta_a-theta_b;
    if dif > pi
        dif2 = -dif2;
    end
    t_diff = sign(dif2) * (dif - sgn(dif-pi)*(dif-pi)*2);

end

function sg = sgn(x)
    if x >= 0
        sg = 1;
    else
        sg = 0;
    end
end