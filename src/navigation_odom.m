%%  NAVIGATION
% This Area is restricted to the NAVIGATION team.
% Only authorized person are admissible to change this content


function [P,x_new,y_new,theta_new,flag_energy, ...
    vel_max,E_budget] = navigation(x_GPS,y_GPS,theta_GPS, ...
    x_past_GPS,y_past_GPS, theta_past_GPS,... 
    x_odom,y_odom,theta_odom, occupancy_grid,P0,v,v_past,E_budget,total_iter, ...
    n_iter, x_new_old, y_new_old,theta_new_old)
    
    Q = eye(3).*0.1;
    R = eye(3).*0.001;

    % % Vector of points for GPS Break Ups
    % GPS_Breakups = [randi([1 size(xt,2)],1,1),randi([1 size(xt,2)],1,1),...
    %                 randi([1 size(xt,2)],1,1),randi([1 size(xt,2)],1,1),...
    %                 randi([1 size(xt,2)],1,1)];
    %             
    Flag = 0;
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
%     H = [((x_new)/Norma) ((y_new)/Norma) 0; ...
%           ((-y_new)/(x_new^2 + y_new^2)) ((x_new)/(x_new^2 + y_new^2)) 0];
    H = [((x_new)/Norma) ((y_new)/Norma) 0; ...
          ((-(y_new-y_new_old))/((x_new-x_new_old)^2 + (y_new-y_new_old)^2)) ((x_new-x_new_old)/((x_new-x_new_old)^2 + (y_new-y_new_old)^2)) 0; ...
          ((x_new)/Norma) ((y_new)/Norma) 0];

    %Norma = norm([IMU_data(i,1) IMU_data(i,2)]);
%     y_hat = [norm([x_new y_new]);atan(y_new/x_new)];
    y_hat = [norm([x_new y_new]);atan2((y_new-y_new_old),(x_new-x_new_old));norm([x_new y_new])];

    % if ismember(i,GPS_Breakups)
    %     Flag = 1;
    % end

    % Check if there is a GPS breakup

    if ~Flag       
        y_theory = [norm([x_GPS y_GPS]);atan2((y_GPS-y_past_GPS),(x_GPS-x_past_GPS));...
                    norm([x_odom y_odom])];

    else
        %% If we decide to use the Prediction Model with odometry
        %y_theory = [norm([IMU_data(i,1) IMU_data(i,2)]);atan(IMU_data(i,2)/IMU_data(i,1))];
        %% If we decide to not use odometry and only use State Model
        % Just update gain and Covariance
    %         K = P*H'/(H*P*H' + H*Q*H');
    %         P = (eye(size(K,1))-K*H)*P;
        Flag = 0;
        return;
    end
    %% Gaussian error of 0.1%
%     u = [(awgn(y_theory(1),abs(y_theory(1)/0.0001),'measured','linear') - y_theory(1)) ...
%         ;(awgn(y_theory(2),abs(y_theory(2)/0.0001),'measured','linear') - y_theory(2))];
    % remove if NaN
%     u(isnan(u)) = 0;
    y = (y_theory - y_hat);
    K = P*H'/(H*P*H' + H*Q*H');
    %% Position Gaussian error
    x_pos = [x_new;y_new;theta_new];

    aux =  x_pos + K*y ;
    P = (eye(size(K,1))-K*H)*P;

    x_new = aux(1);
    y_new = aux(2);
    theta_new = aux(3);

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


%     %% LIDAR
%     % Dimensions of car
%     L = 2.2;
%     Lf = 0.566;
%     dim = L + Lf;
%     resolution = 0.1764;
% 
%     posx_lidar = x_new + dim.*cos(theta_new);
%     posy_lidar = y_new + dim.*sin(theta_new);
% 
%     % create lidar
%     [x_lidar,y_lidar]= lidar();
% 
%     R = [cos(theta_new) -sin(theta_new); sin(theta_new) cos(theta_new)];
%     pos = R*[x_lidar;y_lidar] + [posx_lidar;posy_lidar];
% 
%     for index_laser=1:size(y_lidar,2)
% 
%         % Make sure lidar does not break boundaries
% 
%         if pos(1,index_laser) >= 0.05 && pos(2,index_laser) >= 0.05
% 
%             % Check Occupancy grid
% 
%             if occupancy_grid(round(pos(2,index_laser)/resolution),round(pos(1,index_laser)/resolution)) > 1
%                 disp('Object ahead');
%                 dist_people = sqrt((posx_lidar - pos(1,index_laser))^2 + ...
%                                     (posy_lidar - pos(2,index_laser))^2);
%                 flag_people = 1;
% 
%                 break;
%             end
%         end
% 
%     end


end
