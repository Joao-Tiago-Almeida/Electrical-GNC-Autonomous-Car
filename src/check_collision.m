function flag = check_collision(V_car,theta_car,x0_car,y0_car,V_object,theta_object,x0_object,y0_object)

 % X and Y components for the car's velocity
 Vx_car = V_car*cos(theta_car);
 Vy_car = V_car*sin(theta_car);
 
 % X and Y components for the object's velocity
 Vx_object = V_object*cos(theta_object);
 Vy_object = V_object*sin(theta_object);
 
 x_car = x0_car;
 y_car = y0_car;
 x_object = x0_object;
 y_object = y0_object;
 
 % update position:
 for index_pos =1:10
     
     d = sqrt((x_car - x_object)^2 + (y_car - y_object)^2);
     
     x_car =  x0_car + Vx_car*0.1*index_pos;
     y_car = y0_car + Vy_car*0.1*index_pos;
     
     x_object =  x0_object + Vx_object*0.1*index_pos;
     y_object = y0_object + Vy_object*0.1*index_pos;
     
        
     % The value 0.64 is the width of the car. Since (x_car,y_car) are the
     % coordinates of the point in the middle of the car's front, we need
     % to consider a circunference around this point to cover the possibility
     % of a collision around other points.
     if d < 1.0630-(1.0630/(11-index_pos))
         flag = 1;
         break;
     else
         flag = 0;
     end
 end

end