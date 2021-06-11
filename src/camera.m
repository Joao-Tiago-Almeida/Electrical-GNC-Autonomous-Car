function [x_camera, y_camera] = camera()

% Variables for the camera
N = 4;
b = 8;

% X range
range_1_x = 0.5*ones(1, b + 10*N); range_2_x = 1*ones(1, b + 10*N); range_3_x = 1.5*ones(1, b + 10*N);
range_4_x = 2*ones(1, b + 10*N); range_5_x = 2.5*ones(1, b + 10*N); range_6_x = 3*ones(1, b + 10*N);  
range_7_x = 3.5*ones(1, b + 10*N); range_8_x = 4*ones(1, b + 10*N);  range_9_x = 4.5*ones(1, b + 10*N); 
range_10_x = 5*ones(1, b + 10*N);
range_11_x = 5.5*ones(1, b + 10*N);
range_12_x = 6*ones(1, b + 10*N); range_13_x = 6.5*ones(1, b + 10*N);range_14_x = 7*ones(1, b + 10*N);
range_15_x = 7.5*ones(1, b + 10*N);range_16_x = 8*ones(1, b + 10*N);range_17_x = 8.5*ones(1, b + 10*N);

% Y range
range_11_y = (-4:0.17:4);


x_camera = [range_1_x range_2_x range_3_x range_4_x range_5_x range_6_x range_7_x range_8_x ...
           range_9_x range_10_x range_11_x range_12_x range_13_x range_14_x  range_15_x...
           range_16_x range_17_x]*0.5/0.25*0.1763;
       
y_camera = [range_11_y range_11_y range_11_y range_11_y range_11_y range_11_y range_11_y range_11_y  ...
           range_11_y range_11_y range_11_y range_11_y range_11_y range_11_y range_11_y range_11_y ...
           range_11_y ];

% figure();
% plot(x_camera,y_camera,'g+');

end