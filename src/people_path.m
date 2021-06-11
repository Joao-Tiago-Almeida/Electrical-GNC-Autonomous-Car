function People_Path = people_path(npeople)


global duration_people orientation_people initialPoint_People 

% fixed distance 
distance = 20;

% People velocity
velocity = distance/duration_people(npeople);
vx = velocity*cosd(-orientation_people(npeople));
vy = velocity*sind(-orientation_people(npeople));

% Initial position of each person
People_Path = initialPoint_People(:,npeople);

% Initialize final point
p_final = People_Path;
while true 
    
    % check if the pedestrian has walked 20 meters
    if exist('p_final','var')
        if norm(p_final - initialPoint_People(:,npeople)) >= distance
            break;
        end
    end
    % new position for the pedestrian
    x = p_final(1) + vx*0.1;
    y = p_final(2) + vy*0.1;
    
    p_final = [x;y];   
    
    % save positions in array
    People_Path=[People_Path p_final];
      
end

end