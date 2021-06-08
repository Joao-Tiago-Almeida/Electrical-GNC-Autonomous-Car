function halfGuageDisplay(score)
% HALFGUAGEDISPLAY allows you to generate a half guage display or a
% speedometer display that can be used to indicate a probability score as a
% percentage or a percentage value itself.
% The half guage this function creates is clockwise with lower values at the
% left end of the guage and higher values towards the right end. The half
% guage has a radius of 1.
%
% In this function, half guage is used to represent a score between 0 and 1
% as a percentage. This can be easily adapted to display numbers/scores in
% any range.
%
% USAGE:
%
% halfGuageDisplay(score)
%
% INPUT:
% - score: This is the decimal value of the percentage you want to display.
%
% OUTPUT:
% - none
%
% DEPENDENCIES
% colorGradient.m by  Jose Maria Garcia-Valdecasas Bernal --> This needs to
% be in the same directory as this function
%
% EXAMPLES:
% halfGuageDisplay(0.36) --> to display 36% on the speedometer, this draws
% a half circle with radius 1.
%
% Please see the pictures attached to see the output.
%
% -------------------------------------------------------------------------
% Copyright 2016. Pooja C Narayan, Sensium Healthcare Ltd, Oxford UK
% v:1.0 12 Feb 2016. Initial release.
narginchk(1,1);
clc
if (score > 1)
    disp('Error! Please input a decimal value and retry!');
else if (score < 0)
        disp('Error! The score value should be between 0 and 1');
    else
        % code below draws a half circle (0 to pi) --> 0 to 2*pi draws a full
        % circle
        cla %clear the previous axes if any
        radius = 1; %unit circle
        angle = 0:0.01:pi; % for only a half circle
        x = radius * cos(angle);
        y = radius * sin(angle);
        [grad,~] = colorGradient([1 0 0],[0 0.5 0],length(angle)); %c1 set to red and
        % c2 set to green since I wanted to create a colourmap with red at one end of
        % the spectrum and green at the other, can experiment with any colour
        % values you like!!
        
        colormap(grad)
        z = zeros(size(x));
        col = x;  % This is the color, vary with x in this case.
        surface([x;x],[y;y],[z;z],[col;col],'FaceColor','no','EdgeColor','interp','LineWidth',45); %line width set to 45 since we need a thick line - can change this to anything
        text(x(1),y(1),'100%','FontSize',12)
        text(x(length(angle)),y(length(angle)),'0%','FontSize',12)
        text(x(round(length(angle)/2)),y(round(length(angle)/2)),'50%','FontSize',12)
        axis equal
        axis off
        hold on
        
        % this part draws the line to indicate the score and a string to display
        % the value
        scoreRange = linspace(1,0,length(angle));
        [~,index] = min(abs(scoreRange - score));
        scoreString = strcat(num2str(score*100),'%');
        plot([x(index),0],[y(index),0],'-k','LineWidth',2)
        text(-0.05,-0.1,scoreString,'FontSize',12)
    end
end