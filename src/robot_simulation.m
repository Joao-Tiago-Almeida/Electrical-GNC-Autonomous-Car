function [x,y,theta, phi] = robot_simulation(x_k, y_k, theta_k, v, phi_k, w_phi)
    %direct application of the motion equations explained in the report
    L = 2.2;
    dx = v*cos(theta_k)*0.1;
    dy = v*sin(theta_k)*0.1;
    x = x_k+dx;
    y = y_k+dy;
    dphi = w_phi;
    phi = phi_k+dphi*0.1;
    %regulates the maximum value of phi
    if phi > pi/4
        phi = pi/4;
    elseif phi < -pi/4
        phi = -pi/4;
    end
    dtheta = (v/L)*tan(phi_k);
    theta = theta_k+dtheta*0.1;
end