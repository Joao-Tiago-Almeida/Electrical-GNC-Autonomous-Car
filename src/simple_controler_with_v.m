function [ws, v] = simple_controler_with_v(dx, dy, theta, phi, v, dtheta_in, theta_safe, vel_max, wet, stop, cwalk, person, end_stop)
%     global err_w count_w fixed_sample_rate
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
    if ~exist('end_stop','var')
        end_stop = -1;
    end
    if vel_max < 1
        vel_max = 1;
    end
    %check for wet environment
    if wet
        mu = 0.4;
    else
        mu = 0.7;
    end
    %brake effec in velocity
    brake_acc = mu*9.8;
    acc = 3.3;
    L = 2.2;
    vant = v;
    %the perfect theta to converge
    theta_id = atan2(dy/v, dx/v);
    %the difference between the perfect theta and the car theta
    dtheta_aux = difference_from_theta(wrapToPi(theta_id),wrapToPi(theta));
    %theta difference normalization to converge to the path theta
    dtheta = (dtheta_in + dtheta_aux)/2;
    %phi that minimizes theta error (dtheta)
    phi_id = atan2(L*dtheta, v);
    %phi regularization
    if phi_id > pi/4
        phi_id = pi/4;
    elseif phi_id < -pi/4
        phi_id = -pi/4;
    end
    %make the drive more smooth
    if phi_id == pi/4 && phi < -pi/16
        phi_id = -pi/4;
    elseif phi_id == -pi/4 && phi > pi/16
        phi_id = pi/4;
    end
    %check for stop conditions
    if stop || person
        v = 0;
    %check for crosswalks
    elseif cwalk
        v = 1;
    %check if there is a curve
    elseif abs(theta_safe) < 1e-1 && phi_id < 1e-2
        if dx > 1e-6
            v = 10*dx/cos(theta_id);
        else
            v = 10*dy/sin(theta_id);
        end
    elseif abs(theta_safe) < 0.15 && phi_id < 0.1
        v = 3;
    elseif abs(theta_safe) < 0.2 && phi_id < 0.2
        v = 2;
    else
        v = 1;
    end
    %velocity normalization
    if v > vel_max
        v = vel_max;
    end
    %brakes to final stop
    if end_stop ~= -1
        v = 1;% * end_stop/(2/fixed_sample_rate) + 0.5;
    end
    %brakes, cannot be perfect stop
    if vant - v > brake_acc/10
        v = vant - brake_acc/10;
    end
    if v - vant > acc/10
        v = vant + acc/10;
    end
    if v < 0
        print('Negative Velocity')
    end
    %if there is no speed the wheels should not try to turn
    if v == 0
        ws = 0;
%         count_w = 0;
        return;
    end
%     %derivative of phi
%     err_w = (phi_id-phi);
%     %cumulative ws
%     count_w = count_w + err_w;
%     %wheel turning speed
    ws = (phi_id-phi)*10;%+err_w*10+count_w*10;
    %ws regularization
    if ws > 1.5
        ws = 1.5;
    elseif ws < -1.5
        ws = -1.5;
    end
end