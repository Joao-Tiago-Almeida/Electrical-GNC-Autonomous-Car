function [speedlimit_signal,flag_object_ahead,flag_stop_car,flag_Inerent_collision,flag_passadeira,flag_Person,flag_red_ligth,flag_stopSignal,count1,old_value,path1_not_implemented,path2_not_implemented,x_people1,y_people1,x_people2 ,y_people2 ]...
    = sensors(x,y,theta,dim,x_lidar,y_lidar,x_camera,y_camera,path2_not_implemented,path1_not_implemented,flag_Person,flag_red_ligth,speedlimit_signal,...
    people1,people2,count1,cantos_0,v,flag_stopSignal,flag_Inerent_collision,old_value,x_people1,y_people1,x_people2 ,y_people2,t)
    
    % Map information
    global occupancy_matrix max_velocity map_information orientation_people limit_velocity map_velocity
    
    % Other variables 
    global Ncollision  countstop countgo people_walk time_people 
    
    % Global for plots
    global plot_camera pltpeople1 pltpeople2 plot_lidar s2
    
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
    subplot(s2);
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
    
    
%     prob = a + (b-a).*rand(1,1);
%     out = randsrc(1,1,[0,1;1-prob,prob]);
    count1 = count1 + 1;
    if count1 == 200
        count1=1;
    end
       
    % simulation of the camera
    for index_camera=1:size(y_camera,2) 
                
                if pos_camera(1,index_camera) >=0 && pos_camera(2,index_camera) >= 0
                    if occupancy_matrix(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 2 
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
                    if occupancy_matrix(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 3
                         
                        if sem == 1 
%                             disp('Green light');
                            
                            flag_red_ligth = 0;
                        elseif sem == 2
                                                      
%                              disp('Red light');
                             flag_red_ligth = 1;
                        end
                                                
                    end
                    
                    % Stop signal detected
                    if occupancy_matrix(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 4
                                                  
                      flag_stopSignal = 1;
%                       disp('Stop');
                    end
                    
                    % Person detected
                    if occupancy_matrix(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 5
                                                 
                         flag_Person = 1;
                          x_Person = [x_Person,round(pos_camera(1,index_camera)/resolution)+1];
                          y_Person = [y_Person,round(pos_camera(2,index_camera)/resolution)+1];
                          break;
                    else
                        flag_Person = 0;
                          
                    end
                    % Speed limit
                    if occupancy_matrix(round(pos_camera(2,index_camera)/resolution)+1,round(pos_camera(1,index_camera)/resolution)+1) == 7
                      
                        % Duvida: meter aqui condi�ao if para ver se se
                        % altera a vel max ou nao max>lim -> max=lim
                      sinal_limite = [sinal_limite 1];
%                       disp('Stop');
                    else
                        sinal_limite = [sinal_limite 0];
                    end

                end       
    end
    
    if sum(sinal_limite) >= 1
        max_velocity = limit_velocity/3.6;
        speedlimit_signal = 1;
    else
        max_velocity = map_velocity/3.6;
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
        index_pessoa = index_pessoa + 1;
        if index_pessoa<= 50
            
         
            % Check if the path is in a valid position
            if x_people1(index_pessoa) >= 0 && y_people1(index_pessoa) >= 0
                
                if index_pessoa >= 2 && x_people1(index_pessoa-1) >= 0 && y_people1(index_pessoa-1)>=0
                    occupancy_matrix(round(y_people1(index_pessoa-1)/resolution)+1,round(x_people1(index_pessoa-1)/resolution)+1) = old_value;
                         
                end
                
                pltpeople1 = plot(round(x_people1(index_pessoa)/resolution),round(y_people1(index_pessoa)/resolution),'rX');

                % Save occupancy_matrix value before inserting a person
                old_value = occupancy_matrix(round(y_people1(index_pessoa)/resolution)+1,round(x_people1(index_pessoa)/resolution)+1);

                % Update value of the occupancy grid 
                occupancy_matrix(round(y_people1(index_pessoa)/resolution)+1,round(x_people1(index_pessoa)/resolution)+1) = 5; 

                % Values used to check if there is going to be a collision
                % between the car and the person
                theta_people = theta_people1(index_pessoa);

            end
        else
            occupancy_matrix(round(y_people1(index_pessoa-1)/resolution)+1,round(x_people1(index_pessoa-1)/resolution)+1) = old_value;
            index_pessoa = 0;
            % End of the simulated path for person number 1           
            path1_not_implemented =1;            
        end
        
     end

    % Path for 2� Person 
     if x >= 0 && y >= 0  && path2_not_implemented == 0 
        index_pessoa = index_pessoa + 1;
        if index_pessoa<= 50

            if x_people2(index_pessoa) >= 0 && y_people2(index_pessoa) >= 0 

                if index_pessoa >= 2 &&  x_people2(index_pessoa-1) >= 0 && y_people2(index_pessoa-1) >= 0 
                    occupancy_matrix(round(y_people2(index_pessoa-1)/resolution)+1,round(x_people2(index_pessoa-1)/resolution)+1) = old_value;
                        
                end

                pltpeople2 = plot(round(x_people2(index_pessoa)/resolution)+1,round(y_people2(index_pessoa)/resolution)+1,'rX');
                
                old_value = occupancy_matrix(round(y_people2(index_pessoa)/resolution)+1,round(x_people2(index_pessoa)/resolution)+1);
                occupancy_matrix(round(y_people2(index_pessoa)/resolution)+1,round(x_people2(index_pessoa)/resolution)+1) = 5;
                theta_people = theta_people2(index_pessoa);
                x_people = x_people2(index_pessoa);
                y_people = y_people2(index_pessoa);
                
            end
        else
            occupancy_matrix(round(y_people2(index_pessoa-1)/resolution)+1,round(x_people2(index_pessoa-1)/resolution)+1) = old_value;
            index_pessoa = 0;
            path2_not_implemented = 1;
        end
        
     end
     
     % Person or group of people randomly walking 
     
     for npeople=1:length(orientation_people)
         
         % Mudar isto para o tempo: time_people(npeople)
         if t >= time_people(npeople)/0.1
             
                  
        index_random_people(npeople) = index_random_people(npeople) + 1;
        
        if index_random_people(npeople) <= length(people_walk{npeople}(1,:))

            if people_walk{npeople}(1,index_random_people(npeople)) >= 0 && people_walk{npeople}(2,index_random_people(npeople)) >= 0 

                if index_random_people(npeople) >= 2 &&  people_walk{npeople}(1,index_random_people(npeople)-1) >= 0 && people_walk{npeople}(2,index_random_people(npeople)-1)>= 0 
                    
                    occupancy_matrix(round(people_walk{npeople}(2,index_random_people(npeople)-1)/resolution)+1,round(people_walk{npeople}(1,index_random_people(npeople)-1)/resolution)+1) = old_people(npeople);
                        
                end

                pltpeopleRandom{npeople} = plot(round(people_walk{npeople}(1,index_random_people(npeople))/resolution)+1,round(people_walk{npeople}(2,index_random_people(npeople))/resolution)+1,'rX');
                
                 old_people(npeople) = occupancy_matrix(round(people_walk{npeople}(2,index_random_people(npeople))/resolution)+1,round(people_walk{npeople}(1,index_random_people(npeople))/resolution)+1);
                occupancy_matrix(round(people_walk{npeople}(2,index_random_people(npeople))/resolution)+1,round(people_walk{npeople}(1,index_random_people(npeople))/resolution)+1) = 5;               
                
            end
        else
               
            occupancy_matrix(round(people_walk{npeople}(2,end)/resolution)+1,round(people_walk{npeople}(1,end)/resolution)+1) =  old_people(npeople);
                
        end
        end
                      
        
         
     end
    
                           

    %set flag to 0
    flag_object_ahead=0;
    for index_laser=1:size(y_lidar,2)

                % Make sure lidar does not break boundaries

                if pos(1,index_laser) >= 0 && pos(2,index_laser) >= 0

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

                        % Verificar colis�o futura se for objeto est�tico
                        % -> paredes ou zona fora da estrada
                         
                        if  occupancy_matrix(round(pos(2,index_laser)/resolution)+1,round(pos(1,index_laser)/resolution)+1) == 0
                            x_object = pos(1,index_laser);
                            y_object = pos(2,index_laser);
                            theta_object = 0;                          
                            v_object = 0;  
                            % Prever uma colis�o futura
                            flag_Inerent_collision = check_collision(v,theta,posx_carsFront,posy_carsFront,v_object,theta_object,x_object,y_object);

                            if flag_Inerent_collision
                                break;
                            end
                        end
                        
                        
                    end
                end

    end

    % Check collision 
    if occupancy_matrix(round(cantos(2,1)/resolution),round(cantos(1,1)/resolution)) == 0 || ...
       occupancy_matrix(round(cantos(2,2)/resolution),round(cantos(1,2)/resolution)) == 0 || ...
       occupancy_matrix(round(cantos(2,3)/resolution),round(cantos(1,3)/resolution))== 0  || ...
       occupancy_matrix(round(cantos(2,4)/resolution),round(cantos(1,4)/resolution)) == 0 
       flag_stop_car = 1;
       Ncollision = Ncollision + 1;
    else
       flag_stop_car = 0;
    end



     

end
