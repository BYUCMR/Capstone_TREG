clc
clear

%Read the Joystick

addpath('C:\Users\Nathan Usevitch\Documents\MATLAB\Test_2\HebiRobotics-MatlabInput-87d793d')

joy = HebiJoystick(1)

%%
% addpath('C:\Users\Nathan Usevitch\Documents\MATLAB\HebiRobotics-MatlabInput-87d793d')
% addpath('C:\Users\Nathan Usevitch\Documents\MATLAB\HebiRobotics-MatlabInput-87d793d\src\main\resources\matlab')
addpath(genpath('C:\Users\Nathan Usevitch\Documents\MATLAB\HebiRobotics-MatlabInput-87d793d\src'))
%%
% This script plots the joystick orientation in 3D-space
% Requires a joystick
% Created March 2014 by Eric Cox 
% http://ericcox.me

% NOTE: To stop the program, hit ctrl+c. 
% Otherwise, it will run indefinitely.

% Define joystick ID (if only using 1 joystick, this will likely be '1')
ID = 1
% Create joystick variable
joy=vrjoystick(ID);
% joy = HebiJoystick(1)
% Select the number of time steps to rememeber 
% (i.e., trailing tail length)
n=100;
%%
% Preallocate vectors
kz=zeros(1,n);
ky=zeros(1,n);
kx=zeros(1,n);

% loop until ctrl+c
while true
    Y=axis(joy, 1);     % X-axis is joystick axis 1
    X=axis(joy, 2);     % Y-axis is joystick axis 2
    y=-sin(Y*pi/6);      % Correlate axis to angular orientation of joystick
    x=-sin(X*pi/6);      % See above.
    z=sqrt(1-x^2-y^2);  % Vertical height of joystick by Pythagoras
    
    % Define vector representing current joystick orientation
    z_v = [0 1]*z;
    y_v = [0 1]*y;
    x_v = [0 1]*x;
    
    % Define vectors representing joystick orientation history 
    kz(end+1)=z;
    kz=kz(2:n+1);
    ky(end+1)=y;
    ky=ky(2:n+1);
    kx(end+1)=x;
    kx=kx(2:n+1);   

    % Plot marker to define end of joystick
    plot3(x,y,z,'ro','MarkerSize',10,'MarkerFaceColor','r')
    hold on
    
    % Plot current orientation of joystick
    plot3(x_v,y_v,z_v,'-r','LineWidth',3)
    hold on
    
    % Plot Y-reference vector (it can be hard to tell where things are
    % in 3D plots)
    plot3([.5 -.5],[y y],[z z],'--k')
    hold on
    
    % Plot X-reference vector
    plot3([x x],[.5 -.5],[z z],'--k')
    hold on
    
    % Plot historical data (if n>0)
    plot3(kx,ky,kz,':r');
    hold on
    
    % Plot box around plot perimeter for altitude reference
    plot3([-.5 .5 .5 -.5 -.5],[-.5 -.5 .5 .5 -.5],[z z z z z],'-k')
    hold off
    
    % define plot axes
    axis([-.5 .5 -.5 .5 0 1])
    
    % force the plot
    drawnow;
    
    % delay between plots
    pause(.05)
end




