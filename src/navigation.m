%%  NAVIGATION
% This Area is restricted to the NAVIGATION team.
% Only authorized person are admissible to change this content
% FALTA ALTERAR O THETA!!!

function [P,x_new,y_new,theta_new,flag_energy, ...
    vel_max] = navigation(x_GPS,y_GPS,theta_teo, ...
    x_past_GPS,y_past_GPS,P0,E_budget,total_iter, ...
    n_iter, x_new_old, y_new_old,theta_new_old,Flag_GPS_Breakup,...
    x_past_odom, y_past_odom, x_odom, y_odom)

    %% Energy
    global max_velocity
    delta_t = 0.1;
    P_energy = 1000;

    delta_energy_budget = E_budget;
    if delta_energy_budget<=0
       flag_energy = 1; 
    else
       flag_energy = 0; 
    end
    avail_energy_per_step = delta_energy_budget/(total_iter - n_iter);
    vel_max = avail_energy_per_step/(delta_t)/P_energy;
    
    if vel_max > max_velocity
        vel_max = max_velocity;
    end
%     disp("vel_max:" + vel_max);
    %% Extended Kalman Filter

    Q = eye(3).*0.1;
    R = eye(3).*0.001;
      
    %% Prediction Phase

    %% Process State
%     Norma = norm([x_GPS y_GPS]-[x_past_GPS y_past_GPS]);
    Norma = norm([x_odom y_odom]-[x_past_odom y_past_odom]);
    x_new = x_new_old + Norma*cos(theta_new_old);
    y_new = y_new_old + Norma*sin(theta_new_old);
    theta_new = theta_teo+ theta_teo*(10^(-5))*((rand(1,1) > 0.5)*2 - 1);


    F = [1 0 -Norma*sin(theta_new_old);...
         0 1 Norma*cos(theta_new_old); ...
         0 0 1];

    %% Process Covariance

    P = F*P0*F'+F*R*F';

    %% Update Phase
    
    H = eye(3);

    y_hat = [x_new; y_new; theta_new];


    y_theory = [x_GPS; y_GPS; theta_teo];

    % Check if there is a GPS breakup

    if Flag_GPS_Breakup      
        %% If we decide to use the Prediction Model with odometry
%         y_theory = [norm([x_new y_new]);atan2((y_odom-y_past_odom),(x_odom-x_past_odom))];
        %% If we decide to not use odometry and only use the Prediction Step
        return;
    end
    %% Gaussian error of 0.0001%

    u = [randn(1,1)*0.1;...
     randn(1,1)*0.1;...
     randn(1,1)*0.01];
    % remove if NaN
    u(isnan(u)) = 0;
    y = y_theory + u - y_hat ;
    K = P*H'/(H*P*H' + H*Q*H');
    %% Position Gaussian error
    x_pos = [x_new;y_new;theta_new];

    aux =  x_pos + K*y ;
    P = (eye(size(K,1))-K*H)*P;

    x_new = aux(1)+randn(1,1)*0.05;
    y_new = aux(2)+randn(1,1)*0.05;
    theta_new = aux(3)+randn(1,1)*0.005;
    
    if norm([x_new-x_GPS,y_new-y_GPS]) > 1
        debug = 1;
    end


end
