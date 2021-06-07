function [occupancy_grid,flag_object_ahead,flag_stop_car,flag_Inerent_collision,flag_passadeira,flag_Person,flag_red_ligth,flag_stopSignal,count1,i,old_value,path1_not_implemented,path2_not_implemented,x_people1,y_people1,x_people2 ,y_people2 ]...
    = sensors(x,y,theta,dim,x_lidar,y_lidar,x_camera,y_camera,path2_not_implemented,path1_not_implemented,flag_Person,flag_red_ligth,...
    people1,people2,occupancy_grid,count1,i,cantos_0,resolution,v,flag_stopSignal,flag_Inerent_collision,old_value,x_people1,y_people1,x_people2 ,y_people2 )
    
    global plot_camera countstop countgo pltpeople1 pltpeople2
    if exist('plot_camera','var')
        delete(plot_camera);
    end
    if exist('pltpeople1','var')
        delete(pltpeople1);
    end
    if exist('pltpeople2','var')
        delete(pltpeople2);
    end
    % Person variable Init
    x_Person =[];
    y_Person  = [];
    
    theta_people1 = people1(3,:);
    flag_passadeira=0;
    theta_people2 = people2(3,:);
    a = 0;
    b= 1;
    
    % rotation matrix for the Rigid transformation of the camera and lidar
    % according to the orientation of the car
    
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    % Position of the middle of the cars' front
    posx_carsFront = x + dim.*cos(theta);
    posy_carsFront = y + dim.*sin(theta);
    
    pos = R*[x_lidar;y_lidar] + [posx_carsFront;posy_carsFront];
    pos_camera = R*[x_camera;y_camera] + [posx_carsFront;posy_carsFront];
    
    % update position of 4 corners of the car
    cantos = R*cantos_0 + [x;y];
    
    % Traffic light 
    % if sem is equal to 1 the light is green
    % if sem is equal to 2 the light is red
    if count1>=1 && count1 < 100
     
        sem = 2;          
    elseif count1>=100 && count1<= 200
        sem=1;
    end
    
    
    prob = a + (b-a).*rand(1,1);
    out = randsrc(1,1,[0,1;1-prob,prob]);
    count1 = count1 + 1;
    if count1 == 200
        count1=1;
    end
       
    % simulation of the camera
    for index_camera=1:size(y_camera,2) 
                
                if pos_camera(1,index_camera) >=0 && pos_camera(2,index_camera) >= 0
                    if occupancy_grid(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 2 
                        %If the camera detects a crosswalk we insert a
                        %person 
                        % the variable path1_not_implemented
                        % people1 is a person walking horizontly
                        % Horizontal crossroad 
                        if abs(sin(theta))> 0.5
                            if path1_not_implemented    
                                
                                 x_people1 = people1(1,:) + pos_camera(1,index_camera)+10;
                                 y_people1 =(people1(2,:)) + pos_camera(2,index_camera)+2;

                                 path1_not_implemented = 0;                   
                            end
                        end
                        % people2 is a person walking verticaly
                        % Vertical crossroad
                         if abs(sin(theta))<=0.5
                            if path2_not_implemented    
                                
                                 x_people2 = people2(1,:) + pos_camera(1,index_camera)+2;
                                 y_people2 =(people2(2,:)) + pos_camera(2,index_camera)+10;
                                    
                                 path2_not_implemented = 0;                   
                            end
                        end

                                      
                        flag_passadeira = 1;
                    end                             
                    if occupancy_grid(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 3
                         
                        if sem == 1 
%                             disp('Green light');
                            
                            flag_red_ligth = 0;
                        elseif sem == 2
                                                      
%                              disp('Red light');
                             flag_red_ligth = 1;
                        end
                                                
                    end
                    if occupancy_grid(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 4
                                                  
                      flag_stopSignal = 1;
%                       disp('Stop');
                    end
                    if occupancy_grid(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 5
                                                 
                         flag_Person = 1;
                          x_Person = [x_Person,round(pos_camera(1,index_camera)/resolution)+1];
                          y_Person = [y_Person,round(pos_camera(2,index_camera)/resolution)+1];
                          break;
                    else
                        flag_Person = 0;
                          
                    end

                end       
    end
    
    if countstop < 30 && flag_stopSignal
        flag_stopSignal = 1;
    end
    if countstop >= 30 && countgo < 200 && flag_stopSignal
        flag_stopSignal = 0;
    end
    
    if flag_stopSignal 
        countstop = countstop + 1;
        countgo = 0;
    else
        countgo = countgo + 1;
    end
    
    if countgo == 200% && countstop == 30
        countgo = 0; countstop = 0;
    end
    
    
    % Path for person 1
    if x >= 0 && y >= 0  && path1_not_implemented == 0  
        i = i + 1;
        if i<= 50
            
         
            % Check if the path is in a valid position
            if x_people1(i) >= 0 && y_people1(i) >= 0
                
                if i >= 2 && x_people1(i-1) >= 0 && y_people1(i-1)>=0
                    occupancy_grid(round(y_people1(i-1)/resolution)+1,round(x_people1(i-1)/resolution)+1) = old_value;
                         
                end
                
                pltpeople1 = plot(round(x_people1(i)/resolution),round(y_people1(i)/resolution),'rX');

                % Save occupancy_grid value before inserting a person
                old_value = occupancy_grid(round(y_people1(i)/resolution)+1,round(x_people1(i)/resolution)+1);

                % Update value of the occupancy grid 
                occupancy_grid(round(y_people1(i)/resolution)+1,round(x_people1(i)/resolution)+1) = 5; 

                % Values used to check if there is going to be a collision
                % between the car and the person
                theta_people = theta_people1(i);

            end
        else
            occupancy_grid(round(y_people1(i-1)/resolution)+1,round(x_people1(i-1)/resolution)+1) = old_value;
            i = 0;
            % End of the simulated path for person number 1           
            path1_not_implemented =1;            
        end
        
     end

    % Path for 2� Person 
     if x >= 0 && y >= 0  && path2_not_implemented == 0 
        i = i + 1;
        if i<= 50

            if x_people2(i) >= 0 && y_people2(i) >= 0 

                if i >= 2 &&  x_people2(i-1) >= 0 && y_people2(i-1) >= 0 
                    occupancy_grid(round(y_people2(i-1)/resolution)+1,round(x_people2(i-1)/resolution)+1) = old_value;
                        
                end

                pltpeople2 = plot(round(x_people2(i)/resolution)+1,round(y_people2(i)/resolution)+1,'rX');
                
                old_value = occupancy_grid(round(y_people2(i)/resolution)+1,round(x_people2(i)/resolution)+1);
                occupancy_grid(round(y_people2(i)/resolution)+1,round(x_people2(i)/resolution)+1) = 5;
                theta_people = theta_people2(i);
                x_people = x_people2(i);
                y_people = y_people2(i);
                
            end
        else
            occupancy_grid(round(y_people2(i-1)/resolution)+1,round(x_people2(i-1)/resolution)+1) = old_value;
            i = 0;
            path2_not_implemented = 1;
        end
        
     end

    %set flag to 0
    flag_object_ahead=0;
    for index_laser=1:size(y_lidar,2)

                % Make sure lidar does not break boundaries

                if pos(1,index_laser) >= 0 && pos(2,index_laser) >= 0

                    % Check Occupancy grid

                    if occupancy_grid(round(pos(2,index_laser)/resolution)+1,round(pos(1,index_laser)/resolution)+1) == 0 || ...
                       occupancy_grid(round(pos(2,index_laser)/resolution)+1,round(pos(1,index_laser)/resolution)+1) == 5  
                        
                        
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

                        if flag_Inerent_collision
                            break;
                        end
                    end
                end

    end

    % Check collision 
    if occupancy_grid(round(cantos(2,1)/resolution),round(cantos(1,1)/resolution)) == 0 || ...
       occupancy_grid(round(cantos(2,2)/resolution),round(cantos(1,2)/resolution)) == 0 || ...
       occupancy_grid(round(cantos(2,3)/resolution),round(cantos(1,3)/resolution))== 0  || ...
       occupancy_grid(round(cantos(2,4)/resolution),round(cantos(1,4)/resolution)) == 0 
       flag_stop_car = 1;
    else
       flag_stop_car = 0;
    end


%         h1 = plot(x,y,'bo');
%         h2 = plot(posx_carsFront,posy_carsFront,'ro');
%         h6 = plot(pos_camera(1,:),pos_camera(2,:),'g*');
%         plot(pos(1,end)/resolution,pos(2,end)/resolution,'m*');
%         plot(pos(1,272)/resolution,pos(2,272)/resolution,'m*');
        plot_camera = plot(round(pos_camera(1,:)/resolution)+1,round(pos_camera(2,:)/resolution)+1,'g*');
        
      
%         h7 = plot(cantos(1,:),cantos(2,:),'b');


%         plot(15,5,'y*');
%         plot(8,0,'y*');
% 
%         axis equal;
%         pause(0);
end
