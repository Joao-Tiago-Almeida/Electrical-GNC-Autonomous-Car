function [ws, v] = simple_controler_with_v(dx, dy, theta, phi, v, dtheta_in, theta_safe, vel_max, wet, stop, cwalk, person)
    global err_w  count_w 
    if ~exist('wet','var')
        wet = false;
    end
    if ~exist('stop','var')
        stop = false;
    end
    if ~exist('cwalk','var')
        cwalk = false;
    end
    if ~exist('person','var')
        person = false;
    end
    if wet
        mu = 0.4;
    else
        mu = 0.7;
    end
    brake_acc = mu*9.8;
    L = 2.2;
    vant = v;
    theta_id = wrapToPi(atan2(dy/v, dx/v));
    dtheta_aux = difference_from_theta(wrapToPi(theta_id),wrapToPi(theta));
    dtheta = (dtheta_in + dtheta_aux)/2;
    phi_id = atan2(L*dtheta, v);
    if stop || (person && cwalk) || person
        v = 0;
    elseif cwalk
        v = 1;
    elseif abs(theta_safe) < 1e-4
        if dx > 1e-6
            v = 10*dx/cos(theta_id);
        else
            v = 10*dy/sin(theta_id);
        end
    elseif abs(theta_safe) < 1e-3
        v = 3;
    elseif abs(theta_safe) < 0.2
        v = 2;
    else
        v = 1;
    end
    if v > vel_max
        v = vel_max;
    end
    if vant - v > brake_acc/10
        v = vant - brake_acc/10;
    end
    if v < 0
        print('Negative Velocity')
%         exit(0);
    end
    if phi_id > pi/4
        phi_id = pi/4;
    elseif phi_id < -pi/4
        phi_id = -pi/4;
    end
    if v == 0
        ws = 0;
        return;
    end
    err_w = (phi_id-phi);
    count_w = count_w + err_w;
    ws = (phi_id-phi)*10+err_w*10+count_w*10;
    if ws > 1.5
        ws = 1.5;
    elseif ws < -1.5
        ws = -1.5;
    end
end