% clc; clear; close all
% 
% load('../mat_files/run_points.mat','run_points');
% figure('WindowStyle', 'docked');
% hold on;
% axis equal
% plot(run_points(:,1),run_points(:,2));
% 
% place_car(run_points,15);

function plt = place_car(points,duty_cicle,theta_vect,phi_vect,meter_per_pixel)
    % points : vector of points where the will be displayed
    % duty_cicle : frequency of cars to display
    % theta_vect: vector of the car's angle
    % phi_vect: vector of the steering wheel's angle
    global file_path
    
    if(nargin < 4)
        meter_per_pixel = theta_vect; %third argument is supposed to be the meter_per_pixel
        a = diff(points);
        a = [a; a(end, :)];
        theta_vect = atan2(a(:, 2),a(:,1));
        phi_vect = zeros(size(theta_vect));
    end
    if(nargin<3)
        map_information = load(string(file_path + "map_information.mat"), 'meters_from_MAP');
        meter_per_pixel = map_information.meters_from_MAP;
        a = diff(points);
        a = [a; a(end, :)];
        theta_vect = atan2(a(:, 2),a(:,1));
        phi_vect = zeros(size(theta_vect));
    end


    % car metrics oin meters
    L = 2.2;
    Lr = 0.556;
    Lf = 0.556;
    d = 0.64;
    r = 0.256;
    Length_car = 3.332;
    Width_car = 1.508;
    wheal_offset = (Width_car-2*d)/2;

    % mass center 

    center = [0, 0];
    corners = [-Lr, -d; -Lr, d; L+Lf, d;L+Lf, -d;-Lr, -d];
    wheels = [0,-d-wheal_offset; 0,d+wheal_offset];
    front_wheel = [L, 0; 1, 0];

    car = [center;corners;wheels;front_wheel];

    % car transofmration to the image reference frame (partically unchangable)
    car = car/meter_per_pixel;

    % new transformation
    duty_cicle = min(100,max(1,duty_cicle));
    n_cars = round(100/duty_cicle);
    n_cars = 1:n_cars:size(points,1);
    
    for c = n_cars
        X = points(c,1);
        Y = points(c,2);
        theta=theta_vect(c);
        phi = phi_vect(c);
    
        Rz = @(a)[cos(a) -sin(a); sin(a) cos(a)]';  % this transpose is do to the map of the image has indirect frames

        new_car = car*Rz(theta);
        new_car(1:end-1,:) = new_car(1:end-1,:)+[X Y];

        t_center = new_car(1,:);
        t_corners = new_car(2:6,:);
        t_wheals = new_car(7:8,:);
        t_front_wheel = new_car(9:10,:);

        t_front_wheel(2,:) = t_front_wheel(2,:)*Rz(phi);
        % testing
        hold on

        markers_size = 7;

        plt = plot(t_center(1),t_center(2),'c+',...
                 t_center(1),t_center(2),'gs',...
                 t_corners(:,1),t_corners(:,2),'b-',...
                 t_wheals(:,1),t_wheals(:,2),'k--o',...
                 t_wheals(:,1),t_wheals(:,2),'r*',...
                 t_front_wheel(1,1),t_front_wheel(1,2),'ko',...
                 [t_front_wheel(1,1), t_front_wheel(1,1)+t_front_wheel(2,1)],...
                 [t_front_wheel(1,2), t_front_wheel(1,2)+t_front_wheel(2,2)],'m-*',"MarkerSize",markers_size);


    end
end

%% example of use

% clc; clear; close all
% 
% load('../mat_files/run_points.mat','run_points');
% figure('WindowStyle', 'docked');
% hold on;
% axis equal
% plot(run_points(:,1),run_points(:,2));
% 
% place_car(run_points,20);