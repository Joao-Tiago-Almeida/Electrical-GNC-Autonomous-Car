%%
function [x_lidar,y_lidar]= lidar()
% difference in the number of beams between two consecutive ranges
N = 4;
%initial number of beams in range 1
b = 9;

% coordinate x -> beam -> max of 11 ranges
range_1_x = 0.5*ones(1,b); range_2_x = 1*ones(1,b + N); range_3_x = 1.5*ones(1,b + 2*N);
range_4_x = 2*ones(1, b + 3*N); range_5_x = 2.5*ones(1, b + 4*N); range_6_x = 3*ones(1, b + 5*N);  
range_7_x = 3.5*ones(1, b + 6*N); range_8_x = 4*ones(1, b + 7*N);  range_9_x = 4.5*ones(1, b + 8*N); 
 range_10_x = 5*ones(1, b + 9*N); range_11_x = 5.5*ones(1, b + 10*N); 
 
% coordinate y  -> beam
range_1_y = (-(b-1)/4:0.5:(b-1)/4); range_2_y = (-(b-1)/4 - N/4:0.5:(b-1)/4+N/4); range_3_y = (-(b-1)/4 - N*2/4:0.5:(b-1)/4 +N*2/4);
range_4_y = (-(b-1)/4 - N*3/4:0.5:(b-1)/4 + N*3/4); range_5_y = (-(b-1)/4 - N*4/4:0.5:(b-1)/4 + N*4/4);
range_6_y = (-(b-1)/4 - N*5/4:0.5:(b-1)/4 + N*5/4); range_7_y = (-(b-1)/4 - N*6/4:0.5:(b-1)/4 + N*6/4);
range_8_y = (-(b-1)/4 - N*7/4:0.5:(b-1)/4 + N*7/4); range_9_y = (-(b-1)/4 - N*8/4:0.5:(b-1)/4 + N*8/4);
range_10_y = (-(b-1)/4 - N*9/4:0.5:(b-1)/4 + N*9/4); range_11_y = (-(b-1)/4 - N*10/4:0.5:(b-1)/4 + N*10/4);



% ranges : points in x
x_lidar = [0 range_1_x range_2_x range_3_x range_4_x range_5_x range_6_x range_7_x ...
          range_8_x range_9_x range_10_x range_11_x]*0.5;
       
%  points in y     
y_lidar = [0 range_1_y range_2_y range_3_y range_4_y range_5_y range_6_y range_7_y ...
          range_8_y range_9_y range_10_y range_11_y]*0.2; 
% figure();
% plot(x_lidar,y_lidar,'ro');
end
