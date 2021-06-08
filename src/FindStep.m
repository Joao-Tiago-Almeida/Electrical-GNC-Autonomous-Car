function [b_stp, min_dist, valid] = FindStep(xt, yt, thetat, error)
% This function finds the best stp for simulation. 
% The arguments it expects are:
% xt - the vector with the x coordinates of the path
% yt - the vector with the y coordinates of the path
% thetat - the vector with the thetas of the path
% error - the maximum error expected
% The function should be used like:
% b_stp = FindStep(xt,yt,thetat,error)

    global err_w count_w fixed_sample_rate max_velocity
    min_dist = inf;
    wt = waitbar(0,"Running...");
    for stp = 0.001:0.001:0.1
        err_w = 0;
        count_w = 0;
        vld = 1;
        x = xt(1); y = yt(1);
        x_new = x; y_new = y;
        x_odom = x; y_odom = y;
        t = 0; v = 1; phi = 0;
        end_stop = -1; k_p = 1;
        theta = thetat(1);
        theta_new = theta;
        fin = 0;
        P = [0.01^2 0 0 ; 0 0.01^2 0 ;0 0 (0.01*0.1)^2];
        m_dist = 0;
        while ~fin
            [point, distance, thetap, k_p] = dist_to_traj(x_new, y_new, xt, yt, thetat, v, stp, k_p);
            if distance > m_dist
                m_dist = distance;
            end
            if distance > error
                vld = 0;
                break;
            end
            if length(xt) - k_p < 2/fixed_sample_rate
                end_stop = length(xt)-k_p;
            end
            x_ref = point(1); y_ref = point(2);
            theta_safe = TrackPredict(thetat, fixed_sample_rate, k_p);

            [w_phi, v] = simple_controler_with_v(x_ref-x_new, y_ref-y_new,...
                wrapToPi(theta_new), phi, v,...
                difference_from_theta(wrapToPi(thetap),wrapToPi(theta_new)), theta_safe, max_velocity,...
                false, false, false, false, end_stop);
            x_old = x; y_old = y; v_old = v; x_odom_old = x_odom; y_odom_old = y_odom;
            [x,y,theta,phi] = robot_simulation(x, y, theta, v, phi, w_phi);
            
            x_odom = x_odom+0.005*sin(theta)+(x-x_old);
            y_odom = y_odom+0.005*cos(theta)+(y-y_old);
            
            [P,x_new,y_new,theta_new,~, ...
            ~] = navigation(x,y,theta, ...
            x_old,y_old,P,0,0, 0, x_new, y_new, theta_new, 0,...
            x_odom_old, y_odom_old, x_odom, y_odom);
            t = t + 1;
            if( norm([x-xt(end),y-yt(end)]) < 1)
                fin = 1;
            end
        end
        if rem(stp,0.002)*1e3
            waitbar(stp/0.1,wt,"Running...:P");
        else
            waitbar(stp/0.1,wt,"Running...:D");
        end
        if m_dist < min_dist && m_dist ~= 0 
            min_dist = m_dist;
            b_stp = stp;
            valid = vld;
        end
        if min_dist < error/2
            break;
        end
    end
    close(wt);
end