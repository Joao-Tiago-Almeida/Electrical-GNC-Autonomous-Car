clear;
close all;



load perfect_odom.mat
load pathy.mat
load pathx.mat
load paththeta.mat


global start_v;
%% falta fazer colision detection
%% Simular a matriz do joão
a = linspace(0,10,10/0.0001);
b = reshape(a,[1000 100]);

%% Simulation path
% % Car path
% x_start =b(445,2);y_start=b(445,2);
% [x_teo,y_teo,theta_teo,phi_teo] = testingrobot(x_start,y_start);
% % Person 1 Path
% x_start =b(445,2) - 0.05;y_start=0.8;
% [x_teo1,y_teo1,theta_teo1,phi_teo1] = testingrobot(x_start,y_start);
% 

% figure(30);
% plot(x_teo,y_teo)
% hold on;
% plot(x_teo1,y_teo1)
% legend('True Course','Person Walk');
% legend show;

% IMU_data = [awgn(x_teo',15/0.005,'measured','linear'),awgn(y_teo',15/0.005,'measured','linear'),awgn(theta_teo',15/0.001,'measured','linear')];
IMU_data = perfect_odom';

%% With Timers
% x_new = zeros(size(x_teo,1),1); x_new(1) = x_teo(1);
% y_new = zeros(size(y_teo,1),1); y_new(1) = y_teo(1);
% theta_new = zeros(size(theta_teo,1),1); theta_new(1) = theta_teo(1);

%% For specific simualtions 
x_new = zeros(size(xt,2),1); x_new(1) = xt(1);
y_new = zeros(size(yt,2),1); y_new(1) = yt(1);
theta_new = zeros(size(thetat,2),1); theta_new(1) = thetat(1);

xunc = .01; % 
yunc = .01; % 
Flag = 0;
% Closest neighbour faz-se com o round (cord/1.5)(se as casas tiverem
% desfassadas 1.5 m)(round aproxima casa mais proxima)
Q = eye(3).*0.001;
R = eye(3).*0.001;
P = [xunc^2 0 0 ; 0 xunc^2 0 ;0 0 (xunc*0.1)^2];
x_teo_matrix(1) = x_new(1);
y_teo_matrix(1) = y_new(1);


range_of_radar = 5; %% Number of matrix cells to see 
% Occupancy grid
occ = ones(1000,100);
occ(1:10,:) = 0;
occ(:,1:20) = 0;
occ(:,end) = 0;
occ(:,end-10:end) = 0;
occ(end-10:end,:) = 0;


% Vector of points for GPS Break Ups
GPS_Breakups = [randi([1 size(xt,2)],1,1),randi([1 size(xt,2)],1,1),...
                randi([1 size(xt,2)],1,1),randi([1 size(xt,2)],1,1),...
                randi([1 size(xt,2)],1,1)];

% No relatório falar dos PS Breakups, solução com e sem odmetria.
% Para além disso, testar com mais GPS BreakUps e com eles mais seguidos.
% Em principio, quando estão seguido a odmetria é melhor, caso contrário
% sem odometria ou meter só o barómetro com GPS = 0;
            
for i = 2:size(perfect_odom,2)%size(x_teo,2)
    %% Theoretical Path from Matrix for the Car
    % Using Matrix
%     minMatrix = min(abs(b(:)-x_teo(i)));
%     [row_x(i),col_x(i)] = find(abs(b-x_teo(i))==minMatrix);
%     minMatriy = min(abs(b(:)-y_teo(i)));
%     [row_y(i),col_y(i)] = find(abs(b-y_teo(i))==minMatriy);
%     x_teo_matrix(i) = b(row_x(i),col_x(i));
%     y_teo_matrix(i) = b(row_y(i),col_y(i));

    % For Simulated testing without a Matrix
      x_teo_matrix(i) = xt(i);
      y_teo_matrix(i) = yt(i); 

    %% Theoretical Path from Matrix for the Person
    
%     minMatrix = min(abs(b(:)-x_teo1(i)));
%     [row_x(i),col_x(i)] = find(abs(b-x_teo1(i))==minMatrix);
%     minMatriy = min(abs(b(:)-y_teo1(i)));
%     [row_y(i),col_y(i)] = find(abs(b-y_teo1(i))==minMatriy);
%     xp_teo_matrix(i) = b(row_x(i),col_x(i));
%     yp_teo_matrix(i) = b(row_y(i),col_y(i));
   
%      if occ(row_x(i),col_x(i)) ~= 0
%          occ(row_x(i),col_x(i)) = 100; % Means there is a person
%      end
%      
%      for j = 1:range_of_radar
% %          if()
%           occ(row_x(i)+j,col_x(i));
%           occ(row_x(i)+j,col_x(i)+j);
%           occ(row_x(i)+j,col_x(i)-j);
%      end
%      
     
     %% Prediction Phase

    %% Process State
    Norma = norm([x_teo_matrix(i) y_teo_matrix(i)]-[x_teo_matrix(i-1) y_teo_matrix(i-1)]);
    phi = IMU_data(i,3) - IMU_data(i-1,3);
    x_new(i) = x_new(i-1) + Norma*cos(theta_new(i-1));
    y_new(i) = y_new(i-1) + Norma*sin(theta_new(i-1));
    theta_new(i) = theta_new(i-1) + phi;


    F = [1 0 -Norma*sin(theta_new(i-1));...
         0 1 Norma*cos(theta_new(i-1)); ...
         0 0 1];

    %% Process Covariance

    P = F*P*F'+F*R*F';

    %% Update Phase
    Norma =  norm([x_new(i) y_new(i)]);
    H = [((x_new(i))/Norma) ((y_new(i))/Norma) 0; ...
          ((-y_new(i))/(x_new(i)^2 + y_new(i)^2)) ((x_new(i))/(x_new(i)^2 + y_new(i)^2)) 0];

    %Norma = norm([IMU_data(i,1) IMU_data(i,2)]);
    y_hat = [norm([x_new(i) y_new(i)]);atan(y_new(i)/x_new(i))];
    

    if ismember(i,GPS_Breakups)
        Flag = 1;
    end
    
    % Check if there is a GPS breakup
    
    if ~Flag       
        y_theory = [norm([x_teo_matrix(i) y_teo_matrix(i)]);atan(y_teo_matrix(i)/x_teo_matrix(i))];
    else
        %% If we decide to use the Prediction Model with odometry
        %y_theory = [norm([IMU_data(i,1) IMU_data(i,2)]);atan(IMU_data(i,2)/IMU_data(i,1))];
        %% If we decide to not use odometry and only use State Model
        % Just update gain and Covariance
%         K = P*H'/(H*P*H' + H*Q*H');
%         P = (eye(size(K,1))-K*H)*P;
        Flag = 0;
        continue;
    end
    %% Gaussian error of 0.1%
    u = [(awgn(y_hat(1),abs(y_hat(1)/0.001),'measured','linear') - y_hat(1)) ...
        ;(awgn(y_hat(2),abs(y_hat(2)/0.001),'measured','linear') - y_hat(2))];
    %% remove if NaN
    u(isnan(u)) = 0;
    y = y_theory - y_hat;
    K = P*H'/(H*P*H' + H*Q*H');
    %% Position Gaussian error
    x_pos = [x_new(i);y_new(i);theta_new(i)];
%     w = [(awgn(x_pos(1),abs(x_pos(1)/0.00001),'measured','linear') - x_pos(1)) ...
%         ;(awgn(x_pos(2),abs(x_pos(2)/0.00001),'measured','linear') - x_pos(2)) ...
%         ;(awgn(x_pos(3),abs(x_pos(3)/0.00001),'measured','linear') - x_pos(3))];    
%     w(isnan(w)) = 0;
    aux =  x_pos + K*y ;%+ w;
    P = (eye(size(K,1))-K*H)*P;

    x_new(i) = aux(1);
    y_new(i) = aux(2);
    theta_new(i) = aux(3);
    
%     if occ(row_x(i),col_x(i)) == 100
%          occ(row_x(i),col_x(i)) = 1; % Means there is a person
%      end
     
end
% Position where GPS Breakup Happened
X_breakups = [x_new(GPS_Breakups(1));x_new(GPS_Breakups(2));x_new(GPS_Breakups(3)); ...
               x_new(GPS_Breakups(4)); x_new(GPS_Breakups(5))];
Y_breakups = [y_new(GPS_Breakups(1));y_new(GPS_Breakups(2));y_new(GPS_Breakups(3)); ...
               y_new(GPS_Breakups(4)); y_new(GPS_Breakups(5))];
figure();
plot(x_teo_matrix,y_teo_matrix);
hold on;
plot(IMU_data(:,1),IMU_data(:,2));
hold on;
plot(x_new,y_new);
hold on;
plot(X_breakups,Y_breakups,'O');
legend('True Course','Odometry','EKF Based','GPS breakup Points');
legend show;




% function F = odefcn(t,x)
% Norma = norm([x(1) x(2)]);
% F = [Norma*cos(x(3)*t);
%      Norma*sin(x(3)*t);
%      x(3)];
%  
% end



function [x_vec,y_vec,theta_vec,phi_vec] = testingrobot(x_start,y_start)


start_v = 0;
my_timer = timer('Name', 'my_timer', 'ExecutionMode', 'fixedRate', 'Period', 0.1, ...
                    'StartFcn', @(x,y)disp('started...'), ...
                    'StopFcn', @(x,y)disp('stopped ...'), ...
                    'TimerFcn', @my_start_fcn);
x = x_start;
y = y_start;
t = 0;
theta = pi/4;
x_old = x;
y_old = y;
v = 0.5;
phi = 0;
w_phi = 0.01;
dx = cos(theta)*0.002;
dy = sin(theta)*0.002;
figure
plot([x,x+dx],[y,y+dy],'b');
hold on;
plot(x,y,'O');
start(my_timer);
while t < 50
    if start_v == 1
        x_old = x;
        y_old = y;
        [x,y,theta,phi] = robot_simulation(x, y, theta, v, phi, w_phi);
        dx = cos(theta)*0.002;
        dy = sin(theta)*0.002;
        plot([x,x+dx],[y,y+dy],'b');
        plot(x,y,'O');
        x_vec(t+1) = x;
        y_vec(t+1) = y;
        theta_vec(t+1) = theta;
        phi_vec(t+1) = phi;
        t = t + 1;
        
        start_v = 0;
    end
end



function my_start_fcn(obj, event)
    start_v = 1;
end




end

%% test without Matrix form

%  %% Prediction Phase
% 
%     %% Process State
%     Norma = norm([x_teo(i) y_teo(i)]-[x_teo(i-1) y_teo(i-1)]);
%     x_new(i) = x_new(i-1) + Norma*cos(theta_new(i-1));
%     y_new(i) = y_new(i-1) + Norma*sin(theta_new(i-1));
%     theta_new(i) = IMU_data(i,3);
% 
% 
%     F = [1 0 -Norma*sin(theta_new(i-1));...
%          0 1 Norma*cos(theta_new(i-1)); ...
%          0 0 1];
% 
%     %% Process Covariance
% 
%     P = F*P*F';
% 
%     %% Update Phase
%     Norma =  norm([x_new(i) y_new(i)]);
%     H = [((x_new(i))/Norma) ((y_new(i))/Norma) 0; ...
%           ((-y_new(i))/(x_new(i)^2 + y_new(i)^2)) ((x_new(i))/(x_new(i)^2 + y_new(i)^2)) 0];
% 
%     Norma = norm([IMU_data(i,1) IMU_data(i,2)]);
%     y_hat = [norm([IMU_data(i,1) IMU_data(i,2)] - [x_new(i) y_new(i)]);atan(y_new(i)/x_new(i))];
%     y_theory = [norm([IMU_data(i,1) IMU_data(i,2)] - [x_teo(i) y_teo(i)]);atan(y_teo(i)/x_teo(i))];
%     y = y_theory - y_hat;
%     K = P*H'/(H*P*H' + H*Q*H');
%     aux =  [x_new(i);y_new(i);theta_new(i)] + K*y;
%     P = (eye(size(K,1))-K*H)*P;
% 
%     x_new(i) = aux(1);
%     y_new(i) = aux(2);
%     theta_new(i) = aux(3);
% 
% end
% 
% figure();
% plot(x_teo,y_teo);
% hold on;
% plot(IMU_data(:,1),IMU_data(:,2));
% hold on;
% plot(x_new,y_new);
% legend('True Course','Odometry','EKF Based');
% legend show;

