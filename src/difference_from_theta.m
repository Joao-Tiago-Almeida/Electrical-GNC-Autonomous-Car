function t_diff = difference_from_theta(theta_a, theta_b)
    dif = norm(theta_a-theta_b);

    t_diff = dif - sgn(dif-pi)*(dif-pi)*2;

end

function sg = sgn(x)
    if x >= 0
        sg = 1;
    else
        sg = 0;
    end
end