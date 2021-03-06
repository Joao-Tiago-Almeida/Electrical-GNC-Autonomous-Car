delete(timerfindall)
clear all;
close all hidden;
clc;

%% Guidance

global debug_mode path_points path_orientation map_information file_path occupancy_matrix fixed_sample_rate max_velocity limit_velocity 
global energy_budget map_velocity orientation_people initialPoint_People

debug_mode = false;
create_map

[sampled_path, checkpoints] = path_planning(path_points, path_orientation,"velocity");
max_velocity = map_velocity/3.6; %m/s

if(isempty(sampled_path));return;end

%% Control and Navigation

global Ncollision
PCov = cell(1,5000);
Ncollision=0;
% Timer initialize

global start_v err_w count_w countstop countgo people_walk 


start_v = 0;err_w = 0;count_w = 0;countstop = 0;countgo = 0;

% Convert pixel to meter
initialPoint_People = initialPoint_People*map_information.meters_from_MAP;
Number_of_people =length(orientation_people);
for npeople =1:Number_of_people
    people_walk{npeople} = people_path(npeople);
end


%%

my_timer = timer('Name', 'my_timer', 'ExecutionMode', 'fixedRate', 'Period', 0.01, ...
                    'StartFcn', @(x,y)disp('started...'), ...
                    'StopFcn', @(x,y)disp('stopped...'), ...
                    'TimerFcn', @my_start_fcn);
% Path from guidance
Rini = wrapToPi(deg2rad(path_orientation));

xt = sampled_path(:,1)*map_information.meters_from_MAP;
yt = sampled_path(:,2)*map_information.meters_from_MAP;
thetat = theta_generator(xt,yt);
thetat(1) = -Rini(1); thetat(end) = -Rini(2);

%% Parameters prediction

valid = 0; thderror = 1;
while ~valid && thderror <= 4
    [b_stp, min_dist, valid] = FindStep(xt, yt, thetat, thderror);
    thderror = thderror*2;
end

t_pred = It_Prediction(length(xt));
%% Initialization

% Initialize timer
start(my_timer);
stp = b_stp;
%%
end_stop = -1;
% Initialize Car Exact Position and Old GPS position
x = xt(1);y = yt(1);theta = thetat(1);
x_old = x;y_old = y;theta_old = theta;

% Initialize Pos. estimate 
x_new = x;y_new = y;theta_new = theta;

% Initialize odometry
x_odom = x;y_odom = y;

% Initialize Iterations Counter
t = 0; counter_nav = 0; counter_col = 0; countcol = 0;
fin = 0;

% Initialize Exact Velocity
v = 1;
v_old = v; vel_max = map_velocity;

% Initialize Wheel orientation and angular speed
phi = 0;
w_phi = 0;

% Odometry deviation
error = 0.005;

% Path counter
wait_time = 1;

% colisions
colision = 0;

% Initialize Estimate Covariance of the EKF

P = [0.01^2 0 0 ; 0 0.01^2 0 ;0 0 (0.01*0.1)^2];
E = energy_budget;
P0 = 1000;
Energy_wasted = 0;
flag_energy = 0;
wet = false;

% Sensor variables
load 'Initialize_Sensors.mat';
load 'Initialize_Sensors_flags.mat';
car_stop = 0;

% Create lidar
[x_lidar,y_lidar]= lidar;
% Create camera
[x_camera,y_camera]= camera;
count = 1;
count1 = 1;
count2=1;
count3=1;
speedlimit_signal = 0;
old_Ncollision = 0;

x_people1 = people1(1,:);y_people1=people1(2,:);
x_people2 = people2(1,:);y_people2=people2(2,:);
object_x_old = -1;
object_y_old = -1;

old_value = -1;

% GPS Breakups

% Vector of points for GPS Break Ups
GPS_Breakups = [];
conglomerate_breakups = 1;
    
%% Run the Autonomous Car Program

global s1

h1 = openfig(string(file_path+"MAP.fig"));
ax1 = gca;
fig1 = get(ax1,'children'); %get handle to all the children in the figure
fig2 = get(ax1,'children');

fig = figure("Name","Real Time Simulation",'numbertitle', 'off');
clf;
fig.Position(1) = fig.Position(1)-(fig.Position(3))/2;
fig.Position(3) = 1.7*fig.Position(3);

s1=subplot(2,3,[1,2,4,5]);
copyobj(fig1,s1);
set(gca, 'YDir','reverse')
hold on
box on
axis off
axis equal
title("Circuit");
plot(sampled_path(:,1),sampled_path(:,2),"y--");

s2=subplot(2,3,3);
copyobj(fig2,s2);
set(gca, 'YDir','reverse')
hold on
box on
axis off
axis equal
title("Interest Area");
plot(sampled_path(:,1),sampled_path(:,2),"y--");

s3=subplot(2,3,6);
hold on
title("Speedometer");

close(h1)

wt = waitbar(1,"Energy...");
set(wt,'Name','Energy Variation In Percentage');
 % find the patch object
hPatch = findobj(wt,'Type','Patch');
 % change the edge and face to blue
set(hPatch,'FaceColor','b', 'EdgeColor','w')
tic
while ~fin
    Flag_GPS_Breakup = 0;
    if start_v == 1
        % Measure the distance to tranjectory
        [point, distance, thetap, wait_time] = dist_to_traj(x_new, y_new, xt, yt, thetat, v, stp, wait_time);
        dist_to_p(t+1) = distance;

        if flag_Inerent_collision
            disp("/_\ Inerent colision!")
        end
        if length(xt) - wait_time < 2/fixed_sample_rate
            end_stop = length(xt)-wait_time;
        end
        
        if flag_energy || flag_red_ligth || flag_stopSignal || car_stop
            stopt = true;
        else
            stopt = false;
        end
        
        % Show messages when the camera detects an object
        if flag_red_ligth && count==1
            [icondata,iconcmap] = imread(string(file_path+"sem.jpg")); 
            h=msgbox('Red Light detected',...
         'Camera','custom',icondata,iconcmap);
            count=0;
        elseif flag_passadeira && count==1
            [icondata,iconcmap] = imread(string(file_path+"passa.jpeg")); 
            h=msgbox('Crosswalk detected',...
            'Camera','custom',icondata,iconcmap);
            count=0;
        elseif flag_stopSignal && count ==1
            [icondata,iconcmap] = imread(string(file_path+"stop.jpg")); 
            h=msgbox('Stop Signal detected',...
            'Camera','custom',icondata,iconcmap);
            count=0;
        elseif speedlimit_signal && count==1
            [icondata,iconcmap] = imread(string(file_path+"speed.png")); 
            h=msgbox(sprintf('Speed Limit = %g [k/m]',limit_velocity),...
            'Camera','custom',icondata,iconcmap);
            count=0;
        end
        
        if flag_Person && count2==1
            if exist('h','var')
                delete(h)
            end
            [icondata,iconcmap] = imread(string(file_path+"person.png")); 
            h=msgbox('Person detected',...
            'Camera','custom',icondata,iconcmap);
            count2=0;
        elseif flag_Person==0 &&count2==0
            delete(h)
            count=1;
            count2=1;
        end
        
        if ~isempty(colision) && count3==1
            h1=msgbox(sprintf('Number of collisions so far = %d',colision),...
            'Collision');
            count3=0;
            old_Ncollision = colision;
        elseif colision ~= old_Ncollision
            set(findobj(h1,'Tag','MessageBox'),'String',sprintf('Number of collisions so far = %d',colision));
            old_Ncollision = colision;   
        end
        
        % Controller of the Car
        theta_safe = TrackPredict(thetat, fixed_sample_rate, wait_time);
        [w_phi, v] = simple_controler_with_v(point(1)-x_new, point(2)-y_new,...
            wrapToPi(theta_new), phi, v,...
            difference_from_theta(wrapToPi(thetap),wrapToPi(theta_new)),...
            theta_safe, vel_max, wet, stopt, flag_passadeira||flag_Inerent_collision, flag_Person, end_stop);
        v_aux = v;

        % Car simulator
        [x,y,theta,phi] = robot_simulation(x, y, theta, v, phi, w_phi);
        [stopE, E] = Energy_decreasing(v, v_old, P0, E);
        if E <= 0 && v ~= 0
            disp('No more Energy');
            car_stop = 1;
        elseif E <= 0 && v == 0
            break;
        end

        x_aux = x;
        y_aux = y;
        theta_aux = theta;
        x_odom_old = x_odom;
        y_odom_old = y_odom;
        x_odom = x_odom+error*sin(theta)+(x-x_old);
        y_odom = y_odom+error*cos(theta)+(y-y_old);
        theta_odom = theta;
        
        if round(y/map_information.meters_from_MAP)+1 >0 && round(x/map_information.meters_from_MAP)+1 > 0 && round(y/map_information.meters_from_MAP)+1 <= size(occupancy_matrix,2) && round(x/map_information.meters_from_MAP)+1 <= size(occupancy_matrix,1)
            if randsample( [0 1], 1, true, [0.999 0.001] ) || occupancy_matrix(round(y/map_information.meters_from_MAP)+1,round(x/map_information.meters_from_MAP)+1) == 6
                GPS_Breakups = [GPS_Breakups; t];
                if conglomerate_breakups
                    GPS_Breakups = [GPS_Breakups; (repmat(t,10,1) + (1:1:10)')];
                    conglomerate_breakups = 0;
                end   
            end
        end
        
        if v ~= 0
            counter_nav = counter_nav + 1;
            [P,x_new,y_new,theta_new,flag_energy,vel_max] ...
                = navigation(x,y,theta,x_old,y_old,...
                P,E,t_pred, counter_nav, x_new, y_new, theta_new,any(GPS_Breakups(:) == t),...
                x_odom_old, y_odom_old, x_odom, y_odom);
        end
        % Past GPS position
        x_old = x_aux;
        y_old = y_aux;
        theta_old = theta_aux;
        v_old = v_aux;
        % Save current GPS Position
        vp(t+1) = v;
        wsp(t+1) = w_phi;
        xp(t+1) = x;
        yp(t+1) = y;
        % save Estimation of Car Position
        thetapt(t+1) = theta;
        xnewp(t+1) = x_new;
        ynewp(t+1) = y_new;
        phip(t+1) = phi;
        t = t + 1;
        
        % Lidar Sensors
        
        [speedlimit_signal,flag_object_ahead,flag_stop_car,...
            flag_Inerent_collision,flag_passadeira,flag_Person,...
            flag_red_ligth,flag_stopSignal,count1,old_value,...
            path1_not_implemented,path2_not_implemented,x_people1,...
            y_people1,x_people2 ,y_people2 ] =...
        sensors(x,y,theta,dim,x_lidar,y_lidar,x_camera, ...
                y_camera,path2_not_implemented,path1_not_implemented,...
                flag_Person,flag_red_ligth,speedlimit_signal,...
                people1,people2,count1,cantos_0,v,flag_stopSignal,...
                flag_Inerent_collision,old_value,x_people1,y_people1,...
                x_people2 ,y_people2, t );
        
        error_odom(1,t) = x_odom;
        error_odom(2,t) = y_odom;
        if ~flag_stop_car && exist('crsh','var')
            delete(crsh);
            countcol = 0;
        end
        if flag_stop_car && countcol == 0
            [icondata,iconcmap] = imread(string(file_path+"crash.jpg")); 
            crsh=msgbox('Car crash',...
         'There was a crash','custom',icondata,iconcmap);
            countcol = 1;
            colision = colision + 1;
        elseif flag_stop_car && countcol < 40
            countcol = countcol + 1;
        elseif flag_stop_car && countcol >= 40
            disp('Car has lost himself HELP!!!!');
            break;
        end
        
        start_v = 0;
        if( norm([x-xt(end),y-yt(end)]) < 1)
            fin = 1;
        end
        PCov{t} = P;
    end    
    
    subplot(s1)
    if(t>1); delete(plt1); end
    plt1 = place_car([x/map_information.meters_from_MAP,y/map_information.meters_from_MAP],...
                        100,theta,phi,map_information.meters_from_MAP);
    
    
    subplot(s2)
    if(t>1); delete(plt2); end
    plt2 = place_car([x/map_information.meters_from_MAP,y/map_information.meters_from_MAP],...
                        100,theta,phi,map_information.meters_from_MAP);
    gap = 7;
    xlim([x-gap, x+gap]/map_information.meters_from_MAP)
    ylim([y-gap, y+gap]/map_information.meters_from_MAP)
    
    subplot(s3)
    if v > max_velocity
        v_graph = max_velocity;
    else
        v_graph = v;
    end
    halfGuageDisplay(v_graph/max_velocity);
    
    pause(0.001);
    waitbar(E/energy_budget,wt,sprintf("Energy... %0.2f", (E/energy_budget)*100));
    
    if exist('h','var') && (flag_red_ligth==0 && flag_passadeira==0 && ...
                flag_stopSignal==0 && flag_Person==0 && speedlimit_signal==0)
        delete(h);
        count=1;
        count2=1;
    end
    
end
toc

%% Display final number of collisions
if ~isempty(colision) 
    if exist('h1','var')
        delete(h1);
    end
    h1=msgbox(sprintf('The final number of collisions is = %d',colision),...
    'Collision');    
end

%% Close Energy Display

close(wt);

%% For the Plot of GPS_Breakups

X_breakups = xnewp(GPS_Breakups(:));
Y_breakups = ynewp(GPS_Breakups(:));


%% Timer Stoppage

stop(my_timer);
delete(timerfindall)
clear my_timer       


%% Experimental Results Analysis

figure('WindowStyle', 'docked');
plot(xp,yp,'b'); hold on;
plot(xt,yt,'y'); axis equal;
plot(error_odom(1,:),error_odom(2,:),'r');
plot(xnewp,ynewp,'g');
plot(X_breakups,Y_breakups,'x','MarkerSize',12);
place_car([xp',yp'],1,thetapt,phip,1);
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

%velocity plot
figure('WindowStyle', 'docked');
subplot(2,1,1);
plot(vp);
title('Evolution of car velocity','FontSize',14,'FontName','Arial');
ylabel('Velocity [m/s]','FontSize',12,'FontName','Arial');
xlabel('iterations','FontSize',12,'FontName','Arial');
subplot(2,1,2);
plot(wsp);
title('Evolution of car wheel turning velocity','FontSize',14,'FontName','Arial');
ylabel('Wheel turning speed [rad/s]','FontSize',12,'FontName','Arial');
xlabel('iterations','FontSize',12,'FontName','Arial');

%% Navigation Experimental Analysis

cplot = @(r,x0,y0) plot(x0 + r*cos(linspace(0,2*pi)),y0 + r*sin(linspace(0,2*pi)),'-');
w=1;
for q = 1:t
    if (norm(PCov{q}) < 1)
        cov_propagation(q) = norm(PCov{q});
        w = w+1;
    end
end

figure('WindowStyle', 'docked');
plot(xp,yp,'b'); hold on;
plot(xt,yt,'y'); axis equal;
plot(error_odom(1,:),error_odom(2,:),'r');
plot(xnewp,ynewp,'g');
plot(X_breakups,Y_breakups,'x','MarkerSize',12);
place_car([xp',yp'],1,thetapt,phip,1);

for q = 1:t
    if (norm(PCov{q}) < 1)
        cplot( norm(PCov{q}),xnewp(q),ynewp(q))
        hold on;
    end
end

title('Car Path','FontSize',14,'FontName','Arial');
ylabel('y (m)','FontSize',12,'FontName','Arial');
xlabel('x (m)','FontSize',12,'FontName','Arial');
legend('Actual Car Path','Car Initial Path','Odometry','Position Prediction','GPS BreakUp Points','Covariance Propagation');
legend show;

figure('WindowStyle', 'docked');
subplot(2,1,1)
plot(xp,yp,'b'); hold on;
plot(xt,yt,'y'); axis equal;
plot(error_odom(1,:),error_odom(2,:),'r');
plot(xnewp,ynewp,'g');
plot(X_breakups,Y_breakups,'x','MarkerSize',12);
place_car([xp',yp'],1,thetapt,phip,1);

for q = 1:t
    if (norm(PCov{q}) < 1)
        cplot( norm(PCov{q}),xnewp(q),ynewp(q))
        hold on;
    end
end

title('Car Path','FontSize',14,'FontName','Arial');
ylabel('y (m)','FontSize',12,'FontName','Arial');
xlabel('x (m)','FontSize',12,'FontName','Arial');
legend('Actual Car Path','Car Initial Path','Odometry','Position Prediction','GPS BreakUp Points','Covariance Propagation');
legend show;

subplot(2,1,2)
plot(linspace(1,size(cov_propagation,2),size(cov_propagation,2)), cov_propagation,'--','MarkerSize',12)
hold on;
plot(dist_to_p);
title('Covariance Propagation','FontSize',14,'FontName','Arial');
grid on;
ylabel('Deviation [m]','FontSize',12,'FontName','Arial');
xlabel('Iteration [n]','FontSize',12,'FontName','Arial');
legend('Covariance Propagation','Estimation Error');

%%
disp("Finito")
license('inuse')
[fList,pList] = matlab.codetools.requiredFilesAndProducts('path_planning.m');
%%
function my_start_fcn(obj, event)
    global start_v
    start_v = 1;
end
