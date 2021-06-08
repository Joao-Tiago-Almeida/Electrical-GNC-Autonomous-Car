function People_Path = people_path(npeople)


global duration_people orientation_people initialPoint_people 

% fixed distance 
distance = 20;

% People velocity
velocity = distance/duration_people(npeople);
vx = velocity*cos(orientation_people(npeople));
vy = velocity*sin(orientation_people(npeople));

% Initial position of each person
People_Path = initialPoint_people(:,npeople);

% Initialize final point
p_final = People_Path;
while true 
    
    if exist('p_final','var')
        if norm(p_final - initialPoint_people(:,npeople)) >= distance
            break;
        end
    end
    x = p_final(1) + vx*0.1;
    y = p_final(2) + vy*0.1;
    
    p_final = [x;y];   
    
    People_Path=[People_Path p_final];
      
end

end