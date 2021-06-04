function [ws, v] = simple_controler_with_v(dx, dy, theta, phi, v, dtheta_in, wet, stop, cwalk, person)
    global err_w err_v count_w count_v
%     if nargin<7
%         brake_acc = 7;
%     end
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
    if v == 0
        ws = 0;
        return;
    end
    theta_id = wrapToPi(atan2(dy/v, dx/v));
    dtheta_aux = wrapToPi(theta_id - wrapToPi(theta));
    dtheta = (dtheta_in + dtheta_aux)/2;
    if(abs(dtheta)<count_v)
        count_v = dtheta;
    end
    phi_id = atan2(L*dtheta, v);
    if stop || (person && cwalk)
        v = 0;
    elseif cwalk
        v = 1;
    elseif abs(dtheta) < 1e-2
        if dx > 1e-6
            v = 10*dx/cos(theta);
        else
            v = 10*dy/sin(theta);
        end
    elseif abs(dtheta) < 0.1
        v = 3;
    elseif abs(theta) < 0.2
        v = 2;
    else
        v = 1;
    end
    if v > 5.6
        v = 5.6;
    end
    if vant - v > brake_acc/10
        v = vant - brake_acc/10;
    end
    if(v>err_v)
        err_v = v;
    end
    if v < 0
        print('Negative Velocity')
%         pause
%         exit
    end
%     err_v = dx/cos(theta_id);
%     count_v = count_v + err_v;
    if phi_id > pi/4
        phi_id = pi/4;
    elseif phi_id < -pi/4
        phi_id = -pi/4;
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