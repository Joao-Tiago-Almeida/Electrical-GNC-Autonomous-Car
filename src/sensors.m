function [speedlimit_signal,flag_object_ahead,flag_stop_car,flag_Inerent_collision,flag_passadeira,flag_Person,flag_red_ligth,flag_stopSignal,count1,old_value,path1_not_implemented,path2_not_implemented,x_people1,y_people1,x_people2 ,y_people2 ]...
    = sensors(x,y,theta,dim,x_lidar,y_lidar,x_camera,y_camera,path2_not_implemented,path1_not_implemented,flag_Person,flag_red_ligth,speedlimit_signal,...
    people1,people2,count1,cantos_0,v,flag_stopSignal,flag_Inerent_collision,old_value,x_people1,y_people1,x_people2 ,y_people2,t)
    
    % Map information
    global occupancy_matrix max_velocity map_information orientation_people limit_velocity map_velocity thd_col
    
    % Other variables 
    global Ncollision  countstop countgo people_walk time_people 
    
    % Global for plots
    global plot_camera pltpeople1 pltpeople2 plot_lidar s1
    
    % Variables used only inside this function
    persistent index_pessoa index_random_people pltpeopleRandom old_people 
    
    if isempty(index_pessoa)
        index_pessoa = 0;
    end
    if isempty(index_random_people)
        index_random_people = zeros(1,length(orientation_people));
        old_people = zeros(1,length(orientation_people));
        pltpeopleRandom = cell(1,length(orientation_people));
    end
    resolution = map_information.meters_from_MAP;
    sinal_limite =[];
    subplot(s1);
    if exist('plot_camera','var')
        delete(plot_camera);
    end
    
    if exist('pltpeople1','var')
        delete(pltpeople1);
    end
    
    if exist('pltpeople2','var')
        delete(pltpeople2);
    end
    
    for npeople=1:length((orientation_people))
        if ~isempty(pltpeopleRandom{npeople})
            delete(pltpeopleRandom{npeople});
        end
    end
    
    if exist('plot_lidar','var')
        delete(plot_lidar);
    end
    
    % Person variable Init
    x_Person =[];
    y_Person  = [];
    
    
    flag_passadeira=0;
   
    
    % rotation matrix for the Rigid transformation of the camera and lidar
    % according to the orientation of the car
    
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    % Position of the middle of the cars' front
    posx_carsFront = x + dim.*cos(theta);
    posy_carsFront = y + dim.*sin(theta);
   
    % update the posiiton of the lidar beams and camera applying a rigid
    % body transformation
    %lidar beams
    pos = R*[x_lidar;y_lidar] + [posx_carsFront;posy_carsFront];
    %camera
    pos_camera = R*[x_camera;y_camera] + [posx_carsFront;posy_carsFront];
    
    
    %plot of the camera and lidar 
    plot_camera = plot(round(pos_camera(1,:)/resolution)+1,round(pos_camera(2,:)/resolution)+1,'g*');
    plot_lidar = plot(round(pos(1,:)/resolution)+1,round(pos(2,:)/resolution)+1,'m*');
    
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
    
    % increment counter of traffic light
    count1 = count1 + 1;
    % when the counter reaches 200 the traffic light is reseted
    if count1 == 200
        count1=1;
    end
       
    % simulation of the camera
    for index_camera=1:size(y_camera,2) 
                
                % check if the points in the camera are in a valid position
                % in the map
                if pos_camera(1,index_camera) >=0 && pos_camera(2,index_camera) >= 0 && round(pos_camera(1,index_camera)/resolution)+1 < size(occupancy_matrix,2) && round(pos_camera(2,index_camera)/resolution)+1 < size(occupancy_matrix,1)
                    % if in a point of the camera the occupancy matrix has
                    % a 2, the camera identifies a crosswalk
                    if occupancy_matrix(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 2 
                        %If the camera detects a crosswalk we insert a
                        %person 
                        % the variable path1_not_implemented is set to zero
                        % after the first person is inserted
                        
                        % people1 is a person walking horizontly
                        % Horizontal crossroad 
                        if abs(sin(theta))> 0.5
                            if path1_not_implemented
                                % check if the car is driving from bottom
                                % up or up down
                                
                                % car driving from bottom up
                                if sin(theta) > 0
                                 % fixed position for people 1
                                 x_people1 = people1(1,:) + pos_camera(1,index_camera)+10;
                                 y_people1 =(people1(2,:)) + pos_camera(2,index_camera)+2;
                                 
                                % car driving from up down
                                elseif sin(theta) < 0
                                 % fixed position for people 1 
                                 x_people1 = people1(1,:) + pos_camera(1,index_camera)+10;
                                 y_people1 =(people1(2,:)) + pos_camera(2,index_camera)-2;
                                 
                                end
                                
                                % set this variable to zero or it will
                                % translate the people path to another
                                % point
                                 path1_not_implemented = 0;                   
                            end
                        end
                        % people2 is a person walking verticaly
                        % Vertical crossroad
                         if abs(sin(theta))<=0.5
                             
                            if path2_not_implemented 
                                % check if the car is driving from left to
                                % right or right to left
                                
                                % car driving from left to right
                                if cos(theta) > 0
                                 x_people2 = people2(1,:) + pos_camera(1,index_camera)+2;
                                 y_people2 =(people2(2,:)) + pos_camera(2,index_camera)+7;
                                 
                                 % car driving from right to left
                                elseif cos(theta) < 0
                                 x_people2 = people2(1,:) + pos_camera(1,index_camera)-2;
                                 y_people2 =(people2(2,:)) + pos_camera(2,index_camera)+7;
                                end
                                 % set this variable to zero or it will
                                % translate the people path to another
                                % point
                                 path2_not_implemented = 0;                   
                            end
                        end
                        % this flag is set to one to give to the control group          
                        flag_passadeira = 1;
                    end
                    % if in a point of the camera the occupancy matrix has
                    % a 3, the camera identifies a traffic light
                    if occupancy_matrix(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 3
                        % if the traffic light is green the flag
                        % flag_red_ligth  is set to zero to inform the
                        % control group to keep driving
                        if sem == 1 
%                             disp('Green light');                           
                            flag_red_ligth = 0;
                        % if the traffic light is red the flag
                        % flag_red_ligth  is set to one to inform the
                        % control group to stop
                        elseif sem == 2                                                      
%                              disp('Red light');
                             flag_red_ligth = 1;
                        end
                                                
                    end
                    
                    % Stop signal detected
                    % if in a point of the camera the occupancy matrix has
                    % a 4, the camera identifies a stop signal
                    if occupancy_matrix(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 4
                    % this flag is set to one to inform the control group to stop the car for a while                          
                      flag_stopSignal = 1;
%                       disp('Stop');
                    end
                    
                    % if in a point of the camera the occupancy matrix has
                    % a 5, the camera identifies a pedestrian
                    if occupancy_matrix(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 5
                         % if it detects a person it sets the flag "flag_Person" to one to inform the control group that there is a pedestrian                    
                         flag_Person = 1;
                         % save positions of the pedestrians
                          x_Person = [x_Person,round(pos_camera(1,index_camera)/resolution)+1];
                          y_Person = [y_Person,round(pos_camera(2,index_camera)/resolution)+1];
                          break;
                    else
                        flag_Person = 0;
                          
                    end
                    % Speed limit
                     % if in a point of the camera the occupancy matrix has
                    % a 7, the camera identifies a speed limit signal
                    if occupancy_matrix(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 7
                      % since not every point in a camera detects the speed
                      % limit signal, each time a point detects the signal
                      % it puts the value one in an array and if a point
                      % does not detect the signal then it puts zero.
                      sinal_limite = [sinal_limite 1];
%                       disp('Stop');
                    else
                        sinal_limite = [sinal_limite 0];
                    end

                end       
    end
    
    % if in the array sinal_limite there is at least one indice with value
    % one, then the car is still in the zone of a limited velocity and a
    % flag "speedlimit_signal" is activated to inform the control group
    if sum(sinal_limite) >= 1
        max_velocity = limit_velocity/3.6;
        speedlimit_signal = 1;
    else
        max_velocity = map_velocity/3.6;
    end
    
    % if the camera detected above a stop signal the car stops for 3
    % seconds and then continues driving 
    if countstop < 30 && flag_stopSignal
        flag_stopSignal = 1;
    end
    if countstop >= 30 && countgo < 200 && flag_stopSignal
        flag_stopSignal = 0;
    end
    
    % increment stop signal counter
    if flag_stopSignal 
        countstop = countstop + 1;
        countgo = 0;
    else
        countgo = countgo + 1;
    end
    
    %reset stop signal counter
    if countgo == 200 
        countgo = 0; countstop = 0;
    end
    
    
    % Path for person 1
    % this person has a pre-defined path and walks horizontally
    % if this person has not yet achieved its final position (path1_not_implemented == 0), then it
    % continues walking in the map 
    if x >= 0 && y >= 0  && path1_not_implemented == 0  
        % increment index por person 1
        index_pessoa = index_pessoa + 1;
        
        if index_pessoa<= 50           
         
            % Check if the path is in a valid position
            if x_people1(index_pessoa) >= 0 && y_people1(index_pessoa) >= 0 && round(x_people1(index_pessoa)/resolution)+1 < size(occupancy_matrix,2) && round(y_people1(index_pessoa)/resolution)+1 < size(occupancy_matrix,1)
                
                % delete previous position, which means assigning the old
                % value of the occupancy matrix cell before inserting the
                % person
                if index_pessoa >= 2 && x_people1(index_pessoa-1) >= 0 && y_people1(index_pessoa-1)>=0
                    occupancy_matrix(round(y_people1(index_pessoa-1)/resolution)+1,round(x_people1(index_pessoa-1)/resolution)+1) = old_value;
                         
                end
                % plot the person's path with a red X
                pltpeople1 = plot(round(x_people1(index_pessoa)/resolution),round(y_people1(index_pessoa)/resolution),'rX','MarkerSize',2,'LineWidth',10);

                % Save occupancy_matrix value before inserting a person
                old_value = occupancy_matrix(round(y_people1(index_pessoa)/resolution)+1,round(x_people1(index_pessoa)/resolution)+1);

                % Update value of the occupancy grid to 5 -> this number
                % represents a person
                occupancy_matrix(round(y_people1(index_pessoa)/resolution)+1,round(x_people1(index_pessoa)/resolution)+1) = 5; 

            end
        elseif x_people1(index_pessoa-1) >= 0 && y_people1(index_pessoa-1) >= 0 && round(x_people1(index_pessoa-1)/resolution)+1 < size(occupancy_matrix,2) && round(y_people1(index_pessoa-1)/resolution)+1 < size(occupancy_matrix,1)
            occupancy_matrix(round(y_people1(index_pessoa-1)/resolution)+1,round(x_people1(index_pessoa-1)/resolution)+1) = old_value;
            index_pessoa = 0;
            % End of the simulated path for person number 1           
            path1_not_implemented =1;     
        end
        
     end

    % Path for person 2
    % this person has a pre-defined path and walks vertically
    % if this person has not yet achieved its final position (path2_not_implemented == 0), then it
    % continues walking in the map 
     if x >= 0 && y >= 0  && path2_not_implemented == 0 
        % increment index por person 1
        index_pessoa = index_pessoa + 1;
        
        if index_pessoa<= 50
            
            % Check if the path is in a valid position
            if x_people2(index_pessoa) >= 0 && y_people2(index_pessoa) >= 0 && round(x_people2(index_pessoa)/resolution)+1 < size(occupancy_matrix,2) && round(y_people2(index_pessoa)/resolution)+1 < size(occupancy_matrix,1)
                
                % delete previous position, which means assigning the old
                % value of the occupancy matrix cell before inserting the
                % person
                if index_pessoa >= 2 &&  x_people2(index_pessoa-1) >= 0 && y_people2(index_pessoa-1) >= 0 
                    occupancy_matrix(round(y_people2(index_pessoa-1)/resolution)+1,round(x_people2(index_pessoa-1)/resolution)+1) = old_value;
                        
                end
                % plot the person's path with a red X
                pltpeople2 = plot(round(x_people2(index_pessoa)/resolution)+1,round(y_people2(index_pessoa)/resolution)+1,'rX','MarkerSize',2,'LineWidth',10);
                
                % Save occupancy_matrix value before inserting a person
                old_value = occupancy_matrix(round(y_people2(index_pessoa)/resolution)+1,round(x_people2(index_pessoa)/resolution)+1);
                occupancy_matrix(round(y_people2(index_pessoa)/resolution)+1,round(x_people2(index_pessoa)/resolution)+1) = 5;
                
            end
        elseif x_people2(index_pessoa-1) >= 0 && y_people2(index_pessoa-1) >= 0 && round(x_people2(index_pessoa-1)/resolution)+1 < size(occupancy_matrix,2) && round(y_people2(index_pessoa-1)/resolution)+1 < size(occupancy_matrix,1)
            occupancy_matrix(round(y_people2(index_pessoa-1)/resolution)+1,round(x_people2(index_pessoa-1)/resolution)+1) = old_value;
            index_pessoa = 0;
            % End of the simulated path for person number 2 
            path2_not_implemented = 1;
        end
        
     end
     
     % Person or group of people randomly walking 
     % loop for every person inserted by the user
     for npeople=1:length(orientation_people)
         
         % when the iteration achieves the value inserted by the user, the
         % person starts walking in the map
         if t >= time_people(npeople)/0.1
             
         % increment index and save it in an array        
        index_random_people(npeople) = index_random_people(npeople) + 1;
        
        % check if the path has finished
        if index_random_people(npeople) <= length(people_walk{npeople}(1,:))
            
            %check if the person is in a valid position in the map
            if people_walk{npeople}(1,index_random_people(npeople)) >= 0 && people_walk{npeople}(2,index_random_people(npeople)) >= 0 && round(people_walk{npeople}(1,index_random_people(npeople))/resolution)+1 < size(occupancy_matrix,2) && round(people_walk{npeople}(2,index_random_people(npeople))/resolution)+1 < size(occupancy_matrix,1) 
                
                % delete previous position, which means assigning the old
                % value of the occupancy matrix cell before inserting the
                % person
                if index_random_people(npeople) >= 2 &&  people_walk{npeople}(1,index_random_people(npeople)-1) >= 0 && people_walk{npeople}(2,index_random_people(npeople)-1)>= 0 
                    
                    occupancy_matrix(round(people_walk{npeople}(2,index_random_people(npeople)-1)/resolution)+1,round(people_walk{npeople}(1,index_random_people(npeople)-1)/resolution)+1) = old_people(npeople);                        
                end
                
                % plot the person's path with a red X
                pltpeopleRandom{npeople} = plot(round(people_walk{npeople}(1,index_random_people(npeople))/resolution)+1,round(people_walk{npeople}(2,index_random_people(npeople))/resolution)+1,'rX','MarkerSize',2,'LineWidth',10);
                
                % Save occupancy_matrix value before inserting a person
                old_people(npeople) = occupancy_matrix(round(people_walk{npeople}(2,index_random_people(npeople))/resolution)+1,round(people_walk{npeople}(1,index_random_people(npeople))/resolution)+1);
                % change the value in the occupany matrix cell to 5 ->
                % represents a pedestrian
                occupancy_matrix(round(people_walk{npeople}(2,index_random_people(npeople))/resolution)+1,round(people_walk{npeople}(1,index_random_people(npeople))/resolution)+1) = 5;               
                
            end
        elseif people_walk{npeople}(1,end) >= 0 && people_walk{npeople}(2,end) >= 0 && round(people_walk{npeople}(1,end)/resolution)+1 < size(occupancy_matrix,2) && round(people_walk{npeople}(2,end)/resolution)+1 < size(occupancy_matrix,1) 
               
            occupancy_matrix(round(people_walk{npeople}(2,end)/resolution)+1,round(people_walk{npeople}(1,end)/resolution)+1) =  old_people(npeople);
                
        end
        end
                      
        
         
     end
    
                         

    %set flag to 0
    flag_object_ahead=0;
    % Lidar sensor
    for index_laser=1:size(y_lidar,2)

                % Make sure lidar does not break boundaries

                if pos(1,index_laser) >= 0 && pos(2,index_laser) >= 0 && round(pos(1,index_laser)/resolution)+1 < size(occupancy_matrix,2) && round(pos(2,index_laser)/resolution)+1 < size(occupancy_matrix,1)

                    % Check Occupancy grid
                    % detecting un unknown object or person
                    if occupancy_matrix(round(pos(2,index_laser)/resolution)+1,round(pos(1,index_laser)/resolution)+1) == 0 || ...
                       occupancy_matrix(round(pos(2,index_laser)/resolution)+1,round(pos(1,index_laser)/resolution)+1) == 5  
                   
                        
                        %distance between object and sensor origin
                        distance_to_object = sqrt((posx_carsFront - pos(1,index_laser))^2 + ...
                                            (posy_carsFront - pos(2,index_laser))^2);
                        
                        threshold = 2; % meter
                        if distance_to_object < threshold
                            Ncollision = Ncollision + 1;
%                             plt_collision = plot(round(pos(1,index_laser)/resolution)+1,round(pos(2,index_laser)/resolution)+1,'b*');
                            
                        end
                            
                        flag_object_ahead = 1;

                        % predict if the car is going to collide with the
                        % detected object in the next 10 iterations
                         
                        if  occupancy_matrix(round(pos(2,index_laser)/resolution)+1,round(pos(1,index_laser)/resolution)+1) == 0
                            x_object = pos(1,index_laser);
                            y_object = pos(2,index_laser);
                            theta_object = 0;                          
                            v_object = 0;  
                            
                            flag_Inerent_collision = check_collision(v,theta,posx_carsFront,posy_carsFront,v_object,theta_object,x_object,y_object);
                            
                            %if there is going to be a future collision, a
                            %flag is sent to the control group
                            if flag_Inerent_collision
                                break;
                            end
                        end
                        
                        
                    end
                end

    end

    % Proximity sensors
    % Create a threshold area around the car
    cantos_0_aum = [cantos_0(1,1)-thd_col cantos_0(1,2)-thd_col cantos_0(1,3)+thd_col cantos_0(1,4)+thd_col;...
        cantos_0(2,1)-thd_col cantos_0(2,2)+thd_col cantos_0(2,3)+thd_col cantos_0(2,4)-thd_col];
    cantos1 = R*cantos_0_aum + [x;y];
    
    % function that counts the number of collisions
    flag_stop_car = ColisionsCount(cantos1);

     
end
