%cada pixel tem 0.1764m
clear all
close all
global start_v err_w err_v count_w count_v
start_v = 0;
err_w = 0;
err_v = 0;
count_w = 0;
count_v = inf;
my_timer = timer('Name', 'my_timer', 'ExecutionMode', 'fixedRate', 'Period', 0.01, ...
                    'StartFcn', @(x,y)disp('started...'), ...
                    'StopFcn', @(x,y)disp('stopped...'), ...
                    'TimerFcn', @my_start_fcn);
load('sampled2cm_path.mat');
xt = sampled_path(:,1).*0.1764;
yt = sampled_path(:,2).*0.1764;
thetat = theta_generator(xt,yt);
figure
plot(xt,yt)
hold on;
axis equal;
start(my_timer);
% min_dist1 = inf;
% min_dist2 = inf;
% min_dist3 = inf;
% min_dist4 = inf;
stp = 0.0130;
% for stp = 0.012:0.0001:0.014
x = xt(1);
x_new = x;
xer = 0;
y = yt(1);
y_new = y;
yer = 0;
x_old = x;
y_old = y;
x_odom = x;
y_odom = y;
t = 0;
v = 1;
v_old = v;
phi = 0;
w_phi = 0;
theta = thetat(1);
theta_new = theta;
theta_old = theta;
passe = 10;
error = 0.005;
error_ev = 0;
fin = 0;
m_dist = 0;
m_dist1 = 0;
m_dist2 = 0;
m_dist3 = 0;
m_dist4 = 0;
w_max = 0;
P = [0.01^2 0 0 ; 0 0.01^2 0 ;0 0 (0.01*0.1)^2];
E = 1e6;
P0 = 2000;
Energy_wasted = 0;
flag_energy = 0;
while ~fin
    if start_v == 1
        [point, distance, thetap] = dist_to_traj(x_new, y_new, xt, yt, thetat, v, stp);
        dist_to_p(t+1) = distance;
        if distance > m_dist
            m_dist = distance;
%             elseif y_new < 200 && distance > m_dist2
%                 m_dist2 = distance;
%             elseif y_new < 225 && distance > m_dist4
%                 m_dist4 = distance;
%             elseif distance > m_dist3
%                 m_dist3 = distance;
        end
        if distance > 1
            debug = 1;
            %break;
        end
        x_ref = point(1);
        y_ref = point(2);
        theta_ref = wrapToPi(thetap);
        if flag_energy
            stopt = true;
        else
            stopt = false;
        end
        [w_phi, v] = simple_controler_with_v(x_ref-x_new, y_ref-y_new, theta_new, phi, v, wrapToPi(theta_ref)-wrapToPi(theta_new));%, false, stopt);
        v_aux = v;
        if abs(w_phi) > w_max
            w_max = abs(w_phi);
        end
        [x,y,theta,phi] = robot_simulation(x, y, theta, v, phi, w_phi);
        [stopE, E] = Energy_decreasing(v, v_old, P0, E);
%             if stopE
%                 debug = 1;
%             end
        x_aux = x;
        y_aux = y;
        theta_aux = theta;

        x_odom = x_odom+error*sin(theta)+(x-x_old);
        y_odom = y_odom+error*cos(theta)+(y-y_old);
        theta_odom = theta;

        [P,x_new,y_new,theta_new,flag_energy, ...
            vel_max,Energy_wasted] = navigation(x,y,theta, ...
            x_old,y_old, theta_old,... 
            x_odom,y_odom,theta_odom, [],P,v,v_old,Energy_wasted,3107, t, x_new, y_new, theta_new);

        x_old = x_aux;
        y_old = y_aux;
        theta_old = theta_aux;
        v_old = v_aux;
        xp(t+1) = x;
        yp(t+1) = y;
        thetapt(t+1) = theta;
        xnewp(t+1) = x_new;
        ynewp(t+1) = y_new;
        thetanewp(t+1) = theta_new;
        phip(t+1) = phi;
%         plot(x,y,'O');
        t = t + 1;
%         error_ev = error_ev + error;
%         perfect_odom(1,t) = x;
%         perfect_odom(2,t) = y;
%         perfect_odom(3,t) = theta;
        error_odom(1,t) = x_odom;
        error_odom(2,t) = y_odom;
        error_odom(3,t) = theta_odom;
        start_v = 0;
        if( norm([x-xt(end),y-yt(end)]) < 0.3)
            fin = 1;
        end
    end
%     pause(0.1)
end
%     stp
%     if m_dist1 < min_dist1 && m_dist1 ~= 0 
%         min_dist1 = m_dist1;
%         best_stp1 = stp;
%     end
%     if m_dist2 < min_dist2 && m_dist2 ~= 0
%         min_dist2 = m_dist2;
%         best_stp2 = stp;
%     end
%     if m_dist3 < min_dist3 && m_dist3 ~= 0
%         min_dist3 = m_dist3;
%         best_stp3 = stp;
%     end
%     if m_dist4 < min_dist4 && m_dist4 ~= 0
%         min_dist4 = m_dist4;
%         best_stp4 = stp;
%     end
% end
stop(my_timer);
delete(timerfindall)
clear my_timer
plot(xp,yp,'b');
plot(error_odom(1,:),error_odom(2,:),'r');
plot(xnewp,ynewp,'g');
place_car([xp',yp'],thetapt,phip,30)
figure
plot(dist_to_p);

function my_start_fcn(obj, event)
    global start_v
    start_v = 1;
end