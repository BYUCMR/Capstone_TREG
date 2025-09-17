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

figure('position',[100 100 1000 650])
% Find_Result=gca();
%
%Resample at a fixed frame rate.
xmin=-2;
xmax=2;
ymin=-1;
ymax=2.5;
zmin=-.1;
zmax=3;

for i=1:length(t_movie)
    clf
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
    
    y_vec_R=y_current;
%     hold(Find_Result,'on')
        Plot_Robot_Edges_Color_Spec( Edges_Tube, reshape(y_vec_R,n_all,3), .07, [1 0 0] )
        [Edge_Con, x_conPlot]=Plot_Connections( y_vec_R, T2T_Qual );
        Plot_Robot_Joint_Edges(Edge_Con,x_conPlot,ones(size(T2T_Qual,1),1),.025);
        %Plot the Node/Node Connection Joints with Spheres? 
        Plot_Node_Boxes(y_vec_R, T2T_Qual, d_up, d_down, d_length, d_width*1.2 );
        box off
        grid off
        axis off
        axis equal
        Find_Result=gca();
        
        Ground=[xmin ymin; xmax ymin; xmax ymax; xmin ymax];
        patch(Ground(:,1), Ground(:,2),zmin*ones(length( Ground(:,2)),1),[.9 .9 .9]);


    %Plot the center of mass
    x_com_current=M_Mat*y_current;
    plot3(Find_Result, x_com_current(1,:),x_com_current(2,:),x_com_current(3,:),'k');
%     axis(Find_Result, 'equal');
%     axis(Find_Result, [-2 2 -1 3 0 2.5]); %Set up good axis limits
    
    %Plot the Command in the form of a 3D arrow
    L_arrow=3; %Maximum length of the arrow in the plot 
    
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
%     hold(Find_Result,'off')
    axis([xmin,xmax,ymin,ymax,zmin,zmax])
    lightangle(18,41)
        h.FaceLighting = 'gouraud';
        h.AmbientStrength = 0.3;
        h.DiffuseStrength = 0.8;
        h.SpecularStrength = 0.9;
        h.SpecularExponent = 25;
        h.BackFaceLighting = 'unlit';
    
    
    drawnow
    
    F(i)=getframe(gcf);
end

%%

SAVE_VIDEO=1;
if SAVE_VIDEO==1
    myVideo = VideoWriter('Control_Joy_D_Short_h3.avi');
    % uncompressedVideo = VideoWriter('myfile.avi', 'Uncompressed AVI');
    myVideo.FrameRate = FR;  % Default 30
    myVideo.Quality = 100;    % Default 75
    open(myVideo)
    writeVideo(myVideo,F);
    close(myVideo)
end


