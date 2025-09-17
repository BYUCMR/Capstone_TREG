clc
clear
addpath(genpath(pwd))
%Examine the time until convergence for stacks of Octahedrons to make an
%equivalent move. 

%Time until convergence-- needs to be averaged across different things. 
N_stacks=3;

Master_Order=[4 6 5; 7 9 8; 12 11 10; 13 15 14; 16 18 17];


for Major_Iter=1:N_stacks
    
    if Major_Iter==1
        Nodes=[];
    else
        Nodes=Master_Order(1:Major_Iter-1,:);
    end
    
    %Generate the robot and plot the waypoints
    
    
    in2m=2.54/100
    d_space=2.5*in2m;  %The spacing between the two rollers
    d_offset=10*in2m;   %The distance between the kinematic joint and tube
    d_offset_normal=0*in2m; %The Offset to Move Normal to the plane of the tube
    d_offset_base=2*in2m;   %Offset of how big the box is. 
    L_tube=134*in2m;

    [ x_all, Edges_Tube, Edges_Con, T2T_Qual, B_T, N_subsections, Inds_All, n_true, Edges_True ] = Generate_Shape_Order( Nodes, d_space, d_offset, d_offset_normal, d_offset_base, L_tube );

        n_all=size(x_all,1);
        close all
        Indices_Tube=ones(size(Edges_Tube,1));
        Indices_Con=2*ones(size(Edges_Con,1));
    %     Plot_Robot_Edges(Edges_Tube,reshape(x_all,n_all,3),Indices_Tube, .10);
    %     Plot_Robot_Edges(Edges_Con,reshape(x_all,n_all,3),Indices_Con, .05);
    % %     hold(Find_Result,'on')
    % %     plot3(x_all(1:n,1),x_all(1:n,2),x_all(1:n,3),'o')
    % axis equal

    Order=T2T_Qual;
    L2th=inv(B_T'*B_T)*B_T'; %The psuedo inverse NOT SURE IF THIS IS CORRECT!
    Edges_All=[Edges_Tube; Edges_Con];
    Edges_All_Tube=Edges_Tube;
    %I should plot the passive nodes differently? 
%     Passive=[1,3]; %Note that this is implicit in how I defined the tube routing with Initialize_Octahedron
    for i=1:length(Edges_True)
        Passive(i)=Edges_True{i}(1,1); %Assume the first thing in the cycle is passive.
    end
    n_all=size(x_all,1);
    N_Edges_True=size(Edges_Tube,1);

    %Information for plotting the nodes, doesn't directly control kinematics
        d_up=3.25*in2m;  %Distance from the center of the node
        d_down=1.15*in2m;
        d_length=8/2*in2m;
        d_width=5/2*in2m;

    % Show a Diagram of the Robot with things labeled

    figure
    Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
    hold on
    Number_Nodes( x_all(1:n_true,:));
    axis equal
    title('Nodes')

    %% %Show a Diagram of the Robot with things labeled

    figure
    subplot 121  
    Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
    axis equal
    hold on

    for i=1:size(Edges_All,1)
        Node1=x_all(Edges_All(i,1),:);
        Node2=x_all(Edges_All(i,2),:);
        Center=(Node1+Node2)/2;
        text(Center(1),Center(2),Center(3),num2str(i));
    end
    title('Edges')

    %
    subplot 122
    Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
    hold on
    Number_Nodes( x_all )
    axis equal
    title('Nodes')

        %Record the required time
    %Identify the index of the node in the furthest positive 
    [Val, Ind_t ]=max(x_all(end-2:end,1));
    dist=.5;
    Ind=Ind_t+n_true-3;
    Sign=1;
    if mod(Major_Iter,2)
        Sign=-1;
    end
    Waypoints(1,:)=x_all(Ind,:);
    Waypoints(2,:)=x_all(Ind,:)+[dist, 0, 0];
    Waypoints(3,:)=x_all(Ind,:)+[dist, Sign*dist, 0];
    Waypoints(4,:)=x_all(Ind,:)+[0 Sign*dist 0];
    Waypoints(5,:)=x_all(Ind,:);
    plot3(Waypoints(:,1),Waypoints(:,2), Waypoints(:,3),'-o')  
    
%% Define Contact with the Environment
Support=[1 2 3]; %The Support Polygon Base  This will be the same for all of these. 
% Support=[1, 27, 18]; %For some reason part of this doesn't work, which seems wrong.
C=zeros(6,3*n_all);  %6 Constraints needed in this case. 
C(1,Support(1))=1; %Fix Everything on the First Point
C(2,Support(1)+n_all)=1;
C(3,Support(1)+2*n_all)=1;
C(4,Support(2)+2*n_all)=1;
C(5,Support(2)+n_all)=1;
C(6,Support(3)+2*n_all)=1;
rank(C)
%
C=[];
for i=1:length(Support)
   C_temp=zeros(3,3*n_all);
   C_temp(1,Support(i))=1;
   C_temp(2,Support(i)+n_all)=1;
    C_temp(3,Support(i)+2*n_all)=1;
   C=[C; C_temp];
end


%% Assign Mass Properties

m_meausured=2.83;    %Mass of an active roller module
m_passive=1.6;       %Mass of a passive roller

%Determine the Center of Mass
m_node=ones(1,n_all-n_true)*m_meausured/2; %Assume all other nodes have arbitrary mass
m_kin=zeros(1,n_true); %Assume the joints have no mass

m_tot=[m_kin,m_node];

for it=1:length(Passive)
   Neighbors=find(Edges_Con(:,1)==Passive(it));
   for j=1:length(Neighbors)    %Hardcoding the Fact that Each of these nodes has four neighbros
       m_tot(Edges_Con(Neighbors(j),2))=m_passive/2;       
   end
end

M=(m_tot/sum(m_tot));
CoM=M*x_all;
Z=zeros(1,n_all);

M_Mat=[M, Z, Z; 
       Z, M, Z;
       Z, Z, M];

% y_tot=reshape(x_all,3*n_all,1)';  %Do rows or columns?   Right now I have each time step being a row.

%% Determine the Nominal Length of Each Edge
Lengths=Get_Lengths_E(Edges_All_Tube,x_all);
L_norm=Lengths(1); %The optimum length for the triangles %Set the Default Length

%% Determine the Graph Laplacian of the robots (not just of the points themselves)

N_robots=n_true;  %The number of actual robots

for i=1:N_robots
   First=find(Order(:,1)==i);
   Local{i}=i;
   for j=1:length(First)
        Local{i}=[Local{i} Order(First(j),[2,4])];
   end
%    Row_Coor(i,:)=find(Order(:,1)==i); %Which rows of order coorpesond? 
end

%Generate the Robot Laplacian Elsewhere.


% for i=1:size(Edges_Tube,1)
%         [a, b]=find(Local==Edges_Tube(i,1));
%         [c, d]=find(Local==Edges_Tube(i,2));
%         Edge_R2R(i,:)=[a,c];   %Isn't this just the graph of the initial robot?
% %         Edges_T_L{a}=[Edges_T_L{a}; i];
% %         Edges_T_L{c}=[Edges_T_L{c}; i];
% end
Edge_R2R=[];
for i=1:length(Edges_True)
    Edge_R2R=[Edge_R2R; Edges_True{i}];
end
Adj=zeros(N_robots);
for i=1:size(Edge_R2R,1)
    Adj(Edge_R2R(i,1),Edge_R2R(i,2))=1;
end
Adj=Adj+Adj';
L_robots=diag(sum(Adj))-Adj;    

%% Now perform the actual Dynamics

    %Choose the command to drive towards this.
    speed=1;
    dt=.01;
    y_tot=reshape(x_all,3*n_all,1);  %Initialize the State Vector
    count=1;
    Ind_Control=Ind;
    tic
    for it=1:(size(Waypoints,1)-1)
        dist2target=100; %Make sure we entre the loop the first time
        
        while dist2target>1.1*dt*speed
            x_mat_temp=reshape(y_tot(:,count),n_all,3);
            vec_command=Waypoints(it+1,:)-x_mat_temp(Ind_Control,:);
            Command_Normalized=vec_command'./norm(vec_command)*speed;
            Lock_Node=zeros(3,3*n_all);
            Lock_Node(1,Ind_Control)=1;
            Lock_Node(2,Ind_Control+n_all)=1;
            Lock_Node(3,Ind_Control+2*n_all)=1;
            A_used=Lock_Node(1:3,:);
            b_used=Command_Normalized;

            A=[ A_used; C];  
            b=[ b_used; zeros(size(C,1),1)];

            Loop_Con=[];
            for i=1:N_subsections
                Elements=3;
                Loop_Con=blkdiag(Loop_Con, ones(1,Elements));
            end
            %Measure the time per compute step. 
            
            %Compute the motion using the distributed control
            Cost_Fun_Index=1;
            [ x_opt, Ldot ] = Controled_Motion_Nodes_Dist_Any(y_tot(:,count)', Edges_All, Edges_All_Tube, Loop_Con, L2th, Order, A, b, L_norm, Cost_Fun_Index, L_robots, Local, Passive);
            %Need to look at the evolving constraint 
            
            %Apply the global constraint to all of the columns? 
            
%             A_eq=;
%             b_eq=;
            %Check the results by computing the centralized solution
            
            %Could check the error to centralized solution
%             [ x_opt_cent, Ldot_cent ] = Controled_Motion_Nodes( y_tot(:,count)', Edges_All, Edges_All_Tube, Loop_Con, L2th, Order, A, b, L_norm, Cost_Fun_Index);
%             Cent2Dist_Error=max(abs(x_opt-x_opt_cent))
            
            %Euler Propogation of the Result
            y_tot(:,count+1)=x_opt*dt+y_tot(:,count);
            count=count+1;
            dist2target=norm(y_tot([Ind_Control, Ind_Control+n_all, Ind_Control+2*n_all],count)-Waypoints(it+1,:)');
%             disp(dist2target)
        end
        
    end
    
    %Save some things to be able to generate a movie at the end
    Duration(Major_Iter)=toc
    y_hist{Major_Iter}=y_tot;
    Edges_All_hist{Major_Iter}=Edges_All;
    Waypoints_hist{Major_Iter}= Waypoints;
    N_robots_hist(Major_Iter)=N_robots;
end


%% Generate the Figure
plot(N_robots_hist,Duration,'-o')
xlabel('Number of Nodes')
ylabel('Duration (s)')

%% Generate Schematics of the different ones

%What else could I compute? The number of iterations until convergence? 


%% 
save('Scale_5_Stacks')

%% 
load('Scale_4_Stacks.mat')


%% Show a movie of one configuration.  Can probably upload a movie. 
%Need to show something about the center of mass
figure
for i=1:1:size(y_tot,2)
    x_temp=reshape(y_tot(:,i),n_all,3);
    Plot_Edges(Edges_All, x_temp,'o-')  %All of the Required Edges
    hold on
    plot3(Waypoints(:,1),Waypoints(:,2), Waypoints(:,3),'-o') 
    axis equal
    drawnow
    pause(.1)
    hold off
end
%Could solve any optimization in the framework, but far more efficient to
%solve this one. 

%% Plot the initial and final configuration
figure
Plot_Edges(Edges_All, reshape(y_tot(:,1),n_all,3),'o-')  %All of the Required Edges
hold on
Plot_Edges(Edges_All, reshape(y_tot(:,end),n_all,3),'o-')  %All of the Required Edges
axis equal
plot3(Waypoints(:,1),Waypoints(:,2), Waypoints(:,3),'-o') 



