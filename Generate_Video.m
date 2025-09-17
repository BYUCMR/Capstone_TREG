clc
clear

%Realtime Plotting

%Load the data for plotting
% load('Simulation_Run.mat')
load('Simulation_Run_2.mat')
%Import a vector, and plot it in real time. 

%Determine how to plot these things
FR=15;
t_movie=0:1/FR:max(t_vec);

y_tot_resample=interp1(t_vec,y_tot,t_movie);
x_command_resample=interp1(t_vec,command_hist,t_movie);
%% Play back a video in realtime of the robot moving. 

figure
Find_Result=gca();
%%
%Resample at a fixed frame rate. 
for i=1:length(t_movie)
    
    y_current=y_tot_resample(i,:)'; %(count,:);
    y_current_mat=reshape(y_current,n_all,3);
%     Duration=toc;
%     tic
%     disp(steps_plot/sum(time(end-steps_plot+1:end)))
    %Do I compute the loads just when doing a display?
    
    %Plot the robot (Shape, Loads, CoM, Edge Labels)\
    %Could get the dynamic Loading Data
%     Indices_Tube=L_Load(:,i); %./Max_Load;
%
    
%   In practice, make this update the figure axis in the GUI 
        
    Indices_Tube=ones(12,1);
%     clf
    
%     Find_Result=findobj(obj, 'type', 'axes'); %THis is to plot to the GUI
    %Define the axis
    
    
%     hold(Find_Result,'on')
    Plot_Robot_Edges_h(Edges_All(1:12,:),reshape(y_current',n_all,3),Indices_Tube, .05, Find_Result);
    hold(Find_Result,'on')
    %     hold on
    
%         Plot_Robot_Edges_Color_Spec( Edges_Test(1:12,:), reshape(y_tot(i,:)',n_all,3), .05, [1 0 0] )
        %Plot the Constraint Connections
    [Edge_Con, x_conPlot]=Plot_Connections( y_current', T2T_Qual );
    Plot_Robot_Edges_h(Edge_Con,x_conPlot,ones(size(T2T_Qual,1),1),.025, Find_Result);
        %Plot the Node/Node Connection Joints with Spheres? 
    Plot_Node_Boxes_h(y_current', T2T_Qual, d_up, d_down, d_length, d_width, Passive, Find_Result);
    
    %Plot the center of mass
    x_com_current=M_Mat*y_current;
    plot3(Find_Result, x_com_current(1,:),x_com_current(2,:),x_com_current(3,:),'k');
    axis(Find_Result, 'equal');
    axis(Find_Result, [-2 2 -1 3 0 2.5]); %Set up good axis limits
    
    %Plot the Command in the form of a 3D arrow
    L_arrow=5; %Maximum length of the arrow in the plot 
    
    if Node_Control==7
        Point_Start=x_com_current(:,end);
    else
        Point_Start=y_current_mat(Node_Control,:)';
    end
    Point_End=Point_Start+[x_command_resample(i,:)']*L_arrow/Max_Speed_Node;  %How to rescale command length?
    mArrow3_h(Point_Start, Point_End, Find_Result,'stemWidth',.1,'tipWidth',.15,'color','b','edgecolor','k'); %Plot a 3D Arrow of the Command
    
    %Plot the Center of Mass as a sphere
    [x_sphere,y_sphere,z_sphere]=sphere();
    Size_Sphere=.1;
    surf(Find_Result,Size_Sphere*x_sphere+x_com_current(1,end),Size_Sphere*y_sphere+x_com_current(2,end), Size_Sphere*z_sphere+x_com_current(3,end))
    
    %Number the nodes.  Try this without the nodes
% %     for i=1:6
% %         text(Find_Result, y_current_mat(i,1),y_current_mat(i,2),y_current_mat(i,3),num2str(i),'fontsize',20)
% %     end
    %To control, need a way to orient the robot with respect to the world
    %frame.  Use one IMU to gather that data? Align the Plot?
    hold(Find_Result,'off')
    drawnow
    
    F(i)=getframe(gcf);
end

%%

SAVE_VIDEO=1;
if SAVE_VIDEO==1
    myVideo = VideoWriter('Control.avi');
    % uncompressedVideo = VideoWriter('myfile.avi', 'Uncompressed AVI');
    myVideo.FrameRate = FR;  % Default 30
    myVideo.Quality = 100;    % Default 75
    open(myVideo)
    writeVideo(myVideo,F);
    close(myVideo)
end


