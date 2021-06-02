clear all;
close all;
clc;
%% Guidance

global debug_mode path_points path_orientation

debug_mode = true;
create_map
path_orientation=[0;0];%norte 90, sul -90; este 0graus; etc etc

[sampled_path, checkpoints] = path_planning(path_points, path_orientation);

%% Control and Navigation

% Timer initialize
global start_v  err_w  count_w 
start_v = 0;err_w = 0;count_w = 0;

my_timer = timer('Name', 'my_timer', 'ExecutionMode', 'fixedRate', 'Period', 0.01, ...
                    'StartFcn', @(x,y)disp('started...'), ...
                    'StopFcn', @(x,y)disp('stopped...'), ...
                    'TimerFcn', @my_start_fcn);
% Path from guidance

xt = sampled_path(:,1)*map_information.meters_from_MAP;
yt = sampled_path(:,2)*map_information.meters_from_MAP;
thetat = theta_generator(xt,yt);                

%% Initialization

% Initialize timer
start(my_timer);
stp = 0.02;

% Initialize Car Exact Position and Old GPS position
x = xt(1);y = yt(1);theta = thetat(1);
x_old = x;y_old = y;theta_old = theta;

% Initialize Pos. estimate 
x_new = x;y_new = y;theta_new = theta;

% Initialize odometry
x_odom = x;y_odom = y;

% Initialize Iterations Counter
t = 0;
fin = 0;

% Initialize Exact Velocity
v = 1;
v_old = v;

% Initialize Wheel orientation and angular speed
phi = 0;
w_phi = 0;

% Odometry deviation
error = 0.005;

% Initialize Estimate Covariance of the EKF

P = [0.01^2 0 0 ; 0 0.01^2 0 ;0 0 (0.01*0.1)^2];
E = 1e6;
P0 = 2000;
Energy_wasted = 0;
flag_energy = 0;
wet = false;

% Sensor variables
load 'Initialize_Sensors.mat';
load 'Initialize_Sensors_flags.mat';

% Create lidar
[x_lidar,y_lidar]= lidar;
% Create camera
[x_camera,y_camera]= camera;
count = 1;
index_pessoa=0;

count1 = 1;
object_x_old = -1;
object_y_old = -1;
sem = randsample(1:2,1);
old_value = -1;

% GPS Breakups

% Vector of points for GPS Break Ups
GPS_Breakups = [randi([1 size(xt,2)],1,1),randi([1 size(xt,2)],1,1),...
                randi([1 size(xt,2)],1,1),randi([1 size(xt,2)],1,1),...
                randi([1 size(xt,2)],1,1)];
    
%% Run the Autonomous Car Program
MAP_real_time = load(string(file_path+"MAP.mat"),'MAP');
MAP_real_time.MAP.Name = "O puto tá aí nos drifts -> piu piu";
hold on
plot(sampled_path(:,1),sampled_path(:,2),"y--");

tic
while ~fin
    Flag_GPS_Breakup = 0;
    if start_v == 1
        % Measure the distance to tranjectory
        [point, distance, thetap, wait_time] = dist_to_traj(x_new, y_new, xt, yt, thetat, v, stp);
        dist_to_p(t+1) = distance;
        if distance > 2
            debug = 1
            %break;
        end
        % If the energy is zero, then stop the car
        
        if flag_energy
            stopt = true;
        else
            stopt = false;
        end
        
        %% alteração com a branco e a r
        if flag_Inerent_collision
            disp("colisão da rita a falhar")
        end
        
        if t==1805
            debug = 1;
        end
        
        % Controller of the Car
        
        [w_phi, v] = simple_controler_with_v(point(1)-x_new, point(2)-y_new, wrapToPi(theta_new), phi, v, difference_from_theta(wrapToPi(thetap),wrapToPi(theta_new)), wet, stopt);
        v_aux = v;

        % Car simulator
        [x,y,theta,phi] = robot_simulation(x, y, theta, v, phi, w_phi);
        [stopE, E] = Energy_decreasing(v, v_old, P0, E);

        x_aux = x;
        y_aux = y;
        theta_aux = theta;

        x_odom = x_odom+error*sin(theta)+(x-x_old);
        y_odom = y_odom+error*cos(theta)+(y-y_old);
        theta_odom = theta;
        
%         if ismember(t,GPS_Breakups)
%             Flag_GPS_Breakup = 1;
%         end
%         
        [P,x_new,y_new,theta_new,flag_energy,vel_max,Energy_wasted] ...
            = navigation(x,y,theta,x_old,y_old, theta_old,x_odom,y_odom,theta_odom,...
            P,v,v_old,Energy_wasted,3107, t, x_new, y_new, theta_new,Flag_GPS_Breakup);
        % Past GPS position
        x_old = x_aux;
        y_old = y_aux;
        theta_old = theta_aux;
        v_old = v_aux;
        % Save current GPS Position
        xp(t+1) = x;
        yp(t+1) = y;
        % save Estimation of Car Position
        thetapt(t+1) = theta;
        xnewp(t+1) = x_new;
        ynewp(t+1) = y_new;
        thetanewp(t+1) = theta_new;
        phip(t+1) = phi;
        t = t + 1;
        
        % Lidar Sensors
%         
        [flag_object_ahead,flag_stop_car,flag_Inerent_collision,flag_passadeira,flag_Person,flag_red_ligth,...
            flag_stopSignal,count1,pass_zone_one,pass_zone_two,index_pessoa,sem,old_value]= sensors(x,y,theta,dim,x_lidar,y_lidar,x_camera, ...
            y_camera,pass_zone_one,pass_zone_two,path2_not_implemented,path1_not_implemented,flag_Person,flag_red_ligth,...
            people1,people2,occupancy_matrix,count1,index_pessoa,cantos_0,map_information.meters_from_MAP,v,flag_passadeira,flag_stopSignal,...
            flag_Inerent_collision,sem,old_value);


        error_odom(1,t) = x_odom;
        error_odom(2,t) = y_odom;
        error_odom(3,t) = theta_odom;
        start_v = 0;
        if( norm([x-xt(end),y-yt(end)]) < 0.3)
            fin = 1;
        end
    end    
    if(t>1); delete(plt); end
    plt = place_car([x/map_information.meters_from_MAP,y/map_information.meters_from_MAP],100,theta,phi,map_information.meters_from_MAP);
    
    pause(0.075);
end
toc

%% For the Plot of GPS_Breakups

X_breakups = [xnewp(GPS_Breakups(1));xnewp(GPS_Breakups(2));xnewp(GPS_Breakups(3)); ...
               xnewp(GPS_Breakups(4)); xnewp(GPS_Breakups(5))];
Y_breakups = [xnewp(GPS_Breakups(1));xnewp(GPS_Breakups(2));xnewp(GPS_Breakups(3)); ...
               xnewp(GPS_Breakups(4)); xnewp(GPS_Breakups(5))];


%% Timer Stoppage

stop(my_timer);
delete(timerfindall)
clear my_timer       


%% 
figure('WindowStyle', 'docked');
plot(xp,yp,'b'); hold on;
plot(xt,yt,'y'); axis equal;
plot(error_odom(1,:),error_odom(2,:),'r');
plot(xnewp,ynewp,'g');
plot(X_breakups,Y_breakups,'x');
place_car([xp',yp'],1,thetapt,phip,map_information.meters_from_MAP);
title('Car Path','FontSize',14,'FontName','Arial');
ylabel('y (m)','FontSize',12,'FontName','Arial');
xlabel('x (m)','FontSize',12,'FontName','Arial');
legend('Actual Car Path','Car Initial Path','Odometry','Position Prediction','GPS BreakUp Points');
legend show;

% Error Plot
figure('WindowStyle', 'docked');
plot(dist_to_p);
title('Error of Path','FontSize',14,'FontName','Arial');
ylabel('Error','FontSize',12,'FontName','Arial');
xlabel('iterations','FontSize',12,'FontName','Arial');

%%
MAP_control = load(string(file_path+"MAP.mat"),'MAP');
MAP_control.MAP.Name = 'control';
hold on
place_car([xp',yp']/map_information.meters_from_MAP,3,thetapt,phip,map_information.meters_from_MAP);
plot(sampled_path(:,1),sampled_path(:,2),"y--");
license('inuse')
[fList,pList] = matlab.codetools.requiredFilesAndProducts('path_planning.m');

%%
function my_start_fcn(obj, event)
    global start_v
    start_v = 1;
end