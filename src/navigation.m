%%  NAVIGATION
% This Area is restricted to the NAVIGATION team.
% Only authorized person are admissible to change this content


function [P,x_new,y_new,theta_new,flag_energy, ...
    vel_max,E_budget] = navigation(x_GPS,y_GPS,theta_GPS, ...
    x_past_GPS,y_past_GPS, theta_past_GPS,... 
    x_odom,y_odom,theta_odom,P0,v,v_past,E_budget,total_iter, ...
    n_iter, x_new_old, y_new_old,theta_new_old,Flag_GPS_Breakup)

    %% Energy

    E = 1.378*10^8;
    delta_energy=0;
    M = 810;
    delta_t = 0.1;
    P_energy = 0.01;

    delta_energy = (M*abs((v - v_past)/delta_t)*abs(v) + P_energy)*delta_t;
    E_budget = E_budget + delta_energy;

    % Keep track of the energy spentup to the current instant

    delta_energy_budget = E - E_budget;
    if delta_energy_budget<=0
       flag_energy = 1; 
    else
       flag_energy = 0; 
    end
    avail_energy_per_step = delta_energy_budget/(total_iter - n_iter);
    vel_max = avail_energy_per_step/(delta_t)/P_energy;




    %% Extended Kalman Filter

    Q = eye(3).*0.1;
    R = eye(3).*0.001;
      
    %% Prediction Phase

    %% Process State
    Norma = norm([x_GPS y_GPS]-[x_past_GPS y_past_GPS]);
    x_new = x_new_old + Norma*cos(theta_new_old);
    y_new = y_new_old + Norma*sin(theta_new_old);
    theta_new = theta_odom;


    F = [1 0 -Norma*sin(theta_past_GPS);...
         0 1 Norma*cos(theta_past_GPS); ...
         0 0 1];

    %% Process Covariance

    P = F*P0*F'+F*R*F';

    %% Update Phase
    Norma =  norm([x_new y_new]);

    
    H = [((x_new)/Norma) ((y_new)/Norma) 0; ...
          ((-(y_new-y_new_old))/((x_new-x_new_old)^2 + (y_new-y_new_old)^2)) ((x_new-x_new_old)/((x_new-x_new_old)^2 + (y_new-y_new_old)^2)) 0];

    y_hat = [norm([x_new y_new]);atan2((y_new-y_new_old),(x_new-x_new_old))];


    y_theory = [norm([x_GPS y_GPS]);atan2((y_GPS-y_past_GPS),(x_GPS-x_past_GPS))];

    % Check if there is a GPS breakup

    if Flag_GPS_Breakup      
        %% If we decide to use the Prediction Model with odometry
        %y_theory = [norm([IMU_data(i,1) IMU_data(i,2)]);atan(IMU_data(i,2)/IMU_data(i,1))];
        %% If we decide to not use odometry and only use the Prediction Step
        return;
    end
    %% Gaussian error of 0.0001%

    u = [y_theory(1)*(10^(-6))*((rand(1,1) > 0.5)*2 - 1);y_theory(2)*(10^(-7))*((rand(1,1) > 0.5)*2 - 1)];
    % remove if NaN
     u(isnan(u)) = 0;
    y = y_theory +u - y_hat ;
    K = P*H'/(H*P*H' + H*Q*H');
    %% Position Gaussian error
    x_pos = [x_new;y_new;theta_new];

    aux =  x_pos + K*y ;
    P = (eye(size(K,1))-K*H)*P;

    x_new = aux(1);
    y_new = aux(2);
    theta_new = aux(3);



end
