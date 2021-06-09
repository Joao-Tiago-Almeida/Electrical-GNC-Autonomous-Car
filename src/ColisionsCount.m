function flag_stop_car = ColisionsCount(cantos)
    global map_information occupancy_matrix
    
    flag_stop_car = 0;
    
    %get the car corners
    ct1 = cantos(:,1);
    ct2 = cantos(:,2);
    ct3 = cantos(:,3);
    ct4 = cantos(:,4);
    %get every car limit point
    side_one_x = linspace(ct1(1),ct2(1),ceil(norm(ct1-ct2)/map_information.meters_from_MAP));
    side_one_y = linspace(ct1(2),ct2(2),ceil(norm(ct1-ct2)/map_information.meters_from_MAP));
    side_two_x = linspace(ct2(1),ct3(1),ceil(norm(ct2-ct3)/map_information.meters_from_MAP));
    side_two_y = linspace(ct2(2),ct3(2),ceil(norm(ct2-ct3)/map_information.meters_from_MAP));
    side_three_x = linspace(ct3(1),ct4(1),ceil(norm(ct3-ct4)/map_information.meters_from_MAP));
    side_three_y = linspace(ct3(2),ct4(2),ceil(norm(ct3-ct4)/map_information.meters_from_MAP));
    side_four_x = linspace(ct4(1),ct1(1),ceil(norm(ct4-ct1)/map_information.meters_from_MAP));
    side_four_y = linspace(ct4(2),ct1(2),ceil(norm(ct4-ct1)/map_information.meters_from_MAP));
    
    %carsurrounds
    carsurrounds = [side_one_x side_two_x side_three_x side_four_x;...
        side_one_y side_two_y side_three_y side_four_y];
    
    for k = 1:length(carsurrounds(1,:))
        %0 for not road and 5 for people
        if round(carsurrounds(1,k)/map_information.meters_from_MAP) >= 0 && round(carsurrounds(2,k)/map_information.meters_from_MAP) >= 0 && round(carsurrounds(1,k)/map_information.meters_from_MAP) < size(occupancy_matrix,2) && round(carsurrounds(2,k)/map_information.meters_from_MAP) < size(occupancy_matrix,1)
            if occupancy_matrix(round(carsurrounds(2,k)/map_information.meters_from_MAP),round(carsurrounds(1,k)/map_information.meters_from_MAP)) == 0 || ...
                    occupancy_matrix(round(carsurrounds(2,k)/map_information.meters_from_MAP),round(carsurrounds(1,k)/map_information.meters_from_MAP)) == 5
                flag_stop_car = 1;
                return;
            end
        end
    end
end