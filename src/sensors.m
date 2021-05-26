function [flag_object_ahead,flag_stop_car,flag_Inerent_collision,flag_passadeira,flag_Person,flag_red_ligth,flag_stopSignal,count1,pass_zone_one,pass_zone_two,i,sem,old_value]...
    = sensors(x,y,theta,dim,x_lidar,y_lidar,x_camera,y_camera,pass_zone_one,pass_zone_two,path2_not_implemented,path1_not_implemented,flag_Person,flag_red_ligth,...
    people1,people2,occupancy_grid,count1,i,cantos_0,resolution,v,flag_passadeira,flag_stopSignal,flag_Inerent_collision,sem,old_value)

    % Person variable Init
    x_Person =[];
    y_Person  = [];
    x_people1 = people1(1,:);
    y_people1 = people1(2,:);
    theta_people1 = people1(3,:);
    x_people2 = people2(1,:);
    y_people2 = people2(2,:);
    theta_people2 = people2(3,:);
    a = 0;
    b= 1;
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    posx_carsFront = x + dim.*cos(theta);
    posy_carsFront = y + dim.*sin(theta);
    pos = R*[x_lidar;y_lidar] + [posx_carsFront;posy_carsFront];
    pos_camera = R*[x_camera;y_camera] + [posx_carsFront;posy_carsFront];
    cantos = R*cantos_0 + [x;y];
    prob = a + (b-a).*rand(1,1);
    out = randsrc(1,1,[0,1;1-prob,prob]);
    count1 = count1 + 1;
    if count1 == 100
        count1=1;
    end

    for index_camera=1:size(y_camera,2) 
                if pos_camera(1,index_camera) >=0 && pos_camera(2,index_camera) >= 0
                    if occupancy_grid(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 2 
                        if path2_not_implemented == 1 && pass_zone_one                    
                             x_people2 = x_people2 + pos_camera(1,index_camera)-1;
                             y_people2 = y_people2 + pos_camera(2,index_camera)+1;
                             path2_not_implemented = 0;
                        end
                        if path1_not_implemented == 1           
                             x_people1 = x_people1 + pos_camera(1,index_camera)-1;
                             y_people1 = y_people1 + pos_camera(2,index_camera)+1;
                             path1_not_implemented = 0;                   
                        end
                         prob1 = a + (b-a).*rand(1,1);
                         out1 = randsrc(1,1,[0,1;1-prob1,prob1]);
                         if out1
                              flag_passadeira = 1;
                         else
                             flag_passadeira = 0;
                         end   
                    elseif occupancy_grid(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 3
                         if out 
%                                   x_semaforo = [x_semaforo,round(pos_camera(1,index_camera)/resolution)+1];
%                                   y_semaforo = [y_semaforo,round(pos_camera(2,index_camera)/resolution)+1];
                            if sem ==1 
                                disp('Green light');
                                flag_red_ligth = 0;
                            elseif sem==2
                                 disp('Red light');
                                 flag_red_ligth = 1;
                            end
                         end 
                    elseif occupancy_grid(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 4
                         prob2 = a + (b-a).*rand(1,1);
                         out2 = randsrc(1,1,[0,1;1-prob2,prob2]);
                         if out2 
                              flag_stopSignal = 1;
                              disp('Stop');
%                                   x_stopSignal = [x_stopSignal,round(pos_camera{index}(1,index_camera)/resolution)+1];
%                                   y_stopSignal = [y_stopSignal,round(pos_camera{index}(2,index_camera)/resolution)+1];
                         else
                             flag_stopSignal = 0;
                         end
                    elseif occupancy_grid(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 5
                         prob3 = a + (b-a).*rand(1,1);
                         out3 = randsrc(1,1,[0,1;1-prob3,prob3]);
                         if out3
                             flag_Person = 1;
                              x_Person = [x_Person,round(pos_camera(1,index_camera)/resolution)+1];
                              y_Person = [y_Person,round(pos_camera(2,index_camera)/resolution)+1];
                              theta_people = -pi + (2*pi).*rand(1,1);
%                                   disp('Person');
                         else
                             flag_Person = 0;
                         end
                    end

                end       
    end
    if x >= 0 && y >= 0 && pass_zone_one == 0 && path1_not_implemented ==0  
        i = i + 1;
        if i<= 50

            if x_people1(i) >= 0 && y_people1(i) >= 0

                if i >= 2 && x_people1(i-1) >= 0 && y_people1(i-1)>=0
                    occupancy_grid(round(y_people1(i-1)/resolution)+1,round(x_people1(i-1)/resolution)+1) = old_value;
%                         delete(h5);
                end

%                      h5 = plot(x_people1(i),y_people1(i),'rX');

                % Save occupancy_grid value before inserting a person
                old_value = occupancy_grid(round(y_people1(i)/resolution)+1,round(x_people1(i)/resolution)+1);

                % Update value of the occupancy grid 
                occupancy_grid(round(y_people1(i)/resolution)+1,round(x_people1(i)/resolution)+1) = 5; 

                % Values used to check if there is going to be a collision
                % between the car and the person
                theta_people = theta_people1(i);
%                 x_people = x_people1(i);
%                 y_people = y_people1(i);

            end
        else
            i = 0;
            % End of the simulated path for person number 1
            pass_zone_one = 1;
        end
     end

    % 2º Person 
     if x >= 0 && y >= 0  && pass_zone_two == 0  && path2_not_implemented == 0 
        i = i + 1;
        if i<= 50

            if x_people2(i) >= 0 && y_people2(i) >= 0 

                if i >= 2 &&  x_people2(i-1) >= 0 && y_people2(i-1) >= 0 
                    occupancy_grid(round(y_people2(i-1)/resolution)+1,round(x_people2(i-1)/resolution)+1) = old_value;
%                         delete(h5)
                end
% % 
%                     h5 = plot(x_people2(i),y_people2(i),'rX');
                old_value = occupancy_grid(round(y_people2(i)/resolution)+1,round(x_people2(i)/resolution)+1);
                occupancy_grid(round(y_people2(i)/resolution)+1,round(x_people2(i)/resolution)+1) = 5;
                theta_people = theta_people2(i);
                x_people = x_people2(i);
                y_people = y_people2(i);

            end
        else
            i = 0;
            pass_zone_two = 1;
        end
     end

    %set flag to 0
    flag_object_ahead=0;
    for index_laser=1:size(y_lidar,2)

                % Make sure lidar does not break boundaries

                if pos(1,index_laser) >= 0 && pos(2,index_laser) >= 0

                    % Check Occupancy grid

                    if occupancy_grid(round(pos(2,index_laser)/resolution)+1,round(pos(1,index_laser)/resolution)+1) == 0 || ...
                       occupancy_grid(round(pos(2,index_laser)/resolution)+1,round(pos(1,index_laser)/resolution)+1) >4  

                        % Posição do objeto em pixeis
                        x_object_pixel = round(pos(1,index_laser)/resolution)+1;
                        y_object_pixel = round(pos(2,index_laser)/resolution)+1;
                        % Posição do objeto em metros
                        x_object = pos(1,index_laser);
                        y_object = pos(2,index_laser);

%                         disp('Object ahead');
                        distance_to_objet = sqrt((posx_carsFront - pos(1,index_laser))^2 + ...
                                            (posy_carsFront - pos(2,index_laser))^2);

                        flag_object_ahead = 1;

                        % Verificar se o objeto está estático
%                             if (sum(x_semaforo==x_object_pixel) >=1 && sum(y_semaforo==y_object_pixel)>= 1) || ...
%                                (sum(x_stopSignal==x_object_pixel) >=1 && sum(y_stopSignal==y_object_pixel)>= 1)    
%                                 theta = 0;                          
%                                 v_object = 0;
                        if  (sum(x_Person==x_object_pixel) >=1 && sum(y_Person==y_object_pixel)>= 1) && abs(object_x_old - x_object)>0.25 && abs(object_y_old - y_object)>0.17 
                            theta_object = theta_people;
                            if object_x_old > -1 && object_y_old > -1
                                v_object = sqrt((object_x_old - x_object)^2 + (object_y_old - y_object)^2)/0.1;
                            else
                                v_object = 1;
                            end
                            object_x_old = x_object;
                            object_y_old = y_object;                           
                        else
                            theta_object = 0;                          
                            v_object = 0;
                        end
                        % Calcular a colisão
                        flag_Inerent_collision = check_collision(v,theta,posx_carsFront,posy_carsFront,v_object,theta_object,x_object,y_object);


                    end
                end

    end

    % Check collision 
    if occupancy_grid(round(cantos(2,1)/resolution)+1,round(cantos(1,1)/resolution)+1) == 0 || ...
       occupancy_grid(round(cantos(2,2)/resolution)+1,round(cantos(1,2)/resolution)+1) == 0 || ...
       occupancy_grid(round(cantos(2,3)/resolution)+1,round(cantos(1,3)/resolution)+1)== 0  || ...
       occupancy_grid(round(cantos(2,4)/resolution)+1,round(cantos(1,4)/resolution)+1) == 0 
       flag_stop_car = 1;
    else
       flag_stop_car = 0;
    end


%         h1 = plot(x,y,'bo');
%         h2 = plot(posx_carsFront,posy_carsFront,'ro');
%         h6 = plot(pos_camera(1,:),pos_camera(2,:),'g*');
%         h3 = plot(pos(1,:),pos(2,:),'m*');
%         h7 = plot(cantos(1,:),cantos(2,:),'b');


%         plot(15,5,'y*');
%         plot(8,0,'y*');
% 
%         axis equal;
%         pause(0);
end
