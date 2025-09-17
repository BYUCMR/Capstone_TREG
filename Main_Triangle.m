clc
clear

%Ideas:  Add physical colors to the node connections to make them more
%obvious? 

%The File that Runs the Meat of the simulation while the GUI is active

%Serial communication is enabled through the GUI, I just stream the data 

%% Set up the input method
addpath(genpath(pwd))
in2m=2.54/100;
JOYSTICK_present=0;

if JOYSTICK_present==1
    joy = HebiJoystick(1);
    [axes, buttons, povs] = read(joy);
else
%     kb = HebiKeyboard();
%     keyboardstate= read(kb);
end

% % Test the responsivness of the joystick
% while true
%   [axes, buttons, povs] = read(joy);
%   if any(buttons)
%     disp(['Pressed buttons: ' num2str(find(buttons))]);
% %     disp(axes)
%   end
%   disp(axes)
%   pause(0.1);
% %   print(axes)
% end

%% Launch the GUI (Note the code is elsewhere, all I will do is extract 
%certain values
obj=MorphComGUI
    data=guidata(obj)
%% New Initialization of the robot

%Initialize the Robot

% Nodes=[4 6 5; 7 9 8; 12 11 10; 13 15 14; 16 18 17];
in2m=2.54/100
d_space=2.5*in2m;  %The spacing between the two rollers
d_offset=10*in2m;   %The distance between the kinematic joint and tube
d_offset_normal=0*in2m; %The Offset to Move Normal to the plane of the tube
d_offset_base=2*in2m;   %Offset of how big the box is. 
L_tube=134*in2m;

    d_up=3.25*in2m;  %Distance from the center of the node
    d_down=1.15*in2m;
    d_length=8/2*in2m;
    d_width=5/2*in2m;

x0=[0 0 0; 1 0 0; .5 sqrt(3)/2  0];
% x_all=[0 0 0; 1 0 0; .5 sqrt(3)/2  0];
Edges_Kin{1}=[1 2; 
              2 3;
              3 1];
% x_all
n_true=size(x0,1);
[x_all, Edges_Tube, Edges_Con, T2T_Qual, N_subsections, Inds_All]=Add_Offsets(Edges_Kin, x0, d_space, d_offset, d_offset_normal, d_offset_base, L_tube);
Edges_All=[Edges_Tube; Edges_Con];
Edges_All_Tube=Edges_Tube; %In the triangle case, no extra edges
n_all=size(x_all,1);
N_Edges_True=size(Edges_Tube,1);

Inc_Tri=[1 -1 0;
         0  1 -1;
         -1 0 1];     
%Generate the Incidence th2L matrix for these guys
B_T=[];
for i=1:length(Edges_Kin)
    B_T=blkdiag(B_T,Inc_Tri);
end

%Fix this to be the new version
B_T(:,1:3:size(Edges_All_Tube,1))=[];
L2th=inv(B_T'*B_T)*B_T'; %The psuedo inverse NOT SURE IF THIS IS CORRECT!
figure
Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
hold on
Number_Nodes( x_all(1:n_true,:))
axis equal
title('Nodes')

Passive=1; %The first node is passive
Order=T2T_Qual;
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
    text(Center(1),Center(2),Center(3),num2str(i))
end
title('Edges')

%
subplot 122
Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
hold on
Number_Nodes( x_all )
axis equal
title('Nodes')

%Prepare a Plotting-level Figure

%%

%Define the Support Polygon Individually
C=zeros(6,3*n_all);  %6 Constraints needed in this case. 
C(1,1+2*n_all)=1; %Fix Everything on the First Point
C(2,2+2*n_all)=1; %Fix Everything on the Second Point
C(3,3+2*n_all)=1; %Fix Everything on the Third Point
C(4,1)=1; %Fix the First Point in x
C(5,1+n_all)=1; %Fix the First Point in y 
C(6,2+n_all)=1; %Fix the First Point in x

rank(C)

C_hist(:,:,1)=C;

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

y_tot=reshape(x_all,3*n_all,1)';  %Do rows or columns?   Right now I have each time step being a row.

%% Determine the Nominal Length of Each Edge
Lengths=Get_Lengths_E(Edges_All_Tube,x_all);
L_norm=Lengths(1); %The optimum length for the triangles %Set the Default Length

%% Determine the Graph Laplacian of the robots (not just of the points themselves)

N_robots=n_true;  %The number of actual robots

%This part here is assuming every node has 

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
Edges_True=Edges_Kin;
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



%Generate the "Local" nodes (which nodes are the responsibility of each
%robot)

%% Main Loop
count=1; %A number to count the number of iterations
tic

figure
Node_Pos_All=zeros(1,size(B_T,2));
t_vec=0;

while (1)
  
    %% Extract the Values from the GUI
    data=guidata(obj);
    result=data.popupmenu4;
    Node_Control=result.Value;
    Node_Index = str2num(data.Node_Control_Edit.String);
    Cost_Fun_Index=data.popupmenu5.Value;
    Cent_Decentralized_On=data.popupmenu6.Value; %Distributed or Centralized Computation
    Turn_On_Broadcast=data.broadcast_command.Value;
    XY_Only=data.radiobutton7.Value;
%     Node_Control=0;  %For now Hardcoded (picks the node, or the center of mass)
    
    
    %Possibly also need to set the support index in the GUI
    
    %Get Joystick Commands (or Keyboard commands)
    Command=zeros(3,1);
    Max_Speed_Node=.15;
    if JOYSTICK_present==1
         [axes_joy, buttons, povs] = read(joy);
         Z_command=-axes_joy(3);
         if abs(Z_command)<.15
             Z_command=0;
         end
         Command_joy=[axes_joy(1); -axes_joy(2); Z_command]; %Get the current state of the joystick (Two Axis)

         Command_Normalized=Command_joy*Max_Speed_Node; %Normalize for the maximum speed D
         %Switch the sign of this thing
    else
        Command_Normalized=[.1 0 0]';
    end
  
% Command_Normalized=[.1; 0; 0]; %Hardcode a command    
Command_Normalized=Command_Normalized(1:3); %Take the first two rows 
% disp(Command_Normalized)
% Command_Normalized

%% Determine the Position from the Optimization Routine

%Get the Measurement Values from the Two Nodes, use them in the estimation scheme
%Right now I am only using the tick values. 
tic
for k = 1:K
    for i = 1:N_robots
        p(:, i, k+1) = p(:, i, k) + c_p * x(:, :, k) * L(i, :)'; %Cool computation here
        r{i}(:, k+1) = r{i}(:, k) + c_r * (A_local{i}*x(:, i, k) - b_local{i}); %Duals for local constraint
%         Obj=@(x_var) Obj_Attempt( x_var, i, D, p, r{i}, c_p, c_r, Neighbors, k, x, m, xdes );
%         Obj(x(:,i,k))
        %Note that no gradient information is used here.  Could that
        %improve performance?
        options = optimoptions('fminunc','Display','off','SpecifyObjectiveGradient',true);
        Obj2=@(x_var) Obj_Augmented_L( x_var, Cost{i}, i, p(:,i,k+1), r{i}(:,k+1), c_p, c_r, Neighbors, x(:,:,k), A_local{i}, b_local{i});
        [x_test, val, flag]=fminunc(Obj2,x(:,i,k),options);
        x(:,i,k+1)=x_test;

        % SOme debugging things
%         disp(flag)
%         disp(x_test')
%         disp(r{i}(:,k+1))
%         [xopt,val,flag]=fmincon(Cost{i},x_hat_vec,[],[], Aeq,beq,[],[],[],options);
%         
%         x(:, i, k+1) = invM(:, :, i) * (2 * c_r * A_local{i}'*b_local{i} ...
%             - p(:, i, k+1) ...
%             - A_local{i}'*r{i}(:, k+1)  ...
%             +c_p * x(:, :, k) * absL(i, :)'); %The linear update
%         J(i, k+1) = x(:, i, k+1)'*(D'*D)*x(:, i, k+1); %Compute Centralized Cost at each node
    end
end
Duration=toc


    %% Frame the Control problem
    A_com=M_Mat(1:3,:);
    % D=.15;  %A Value of .2 seems to approach instability .15 works well with
    % only the com constrained
    
    %Also use the ground constraint matrix
%     C_Lock=zeros(9,3*n_all);
%     for i=1:length(Support)
%             C_Lock(3*(i-1)+1,Support(i))=1;
%             C_Lock(3*(i-1)+2,Support(i)+n_all)=1;
%             C_Lock(3*(i-1)+3,Support(i)+2*n_all)=1;
%     end
    
    switch  Node_Control
        
        case 2  %Center of Mass Control
             A_used=A_com;
             b_used=Command_Normalized;
        case 3    %Do nothing, allow the robot to return to equilibrium.
            A_used=[];
            b_used=[];
        otherwise     %Moving a Specific Node
            Lock_Node=zeros(3,3*n_all);
            Lock_Node(1,Node_Index)=1;
            Lock_Node(2,Node_Index+n_all)=1;
            Lock_Node(3,Node_Index+2*n_all)=1;
            if XY_Only==0
                A_used=Lock_Node(1:3,:);
                b_used=Command_Normalized;
            else
                A_used=Lock_Node(1:2,:);
                b_used=Command_Normalized(1:2);
            end
    end

%     if Node_Control==7  %Center of Mass Control
%         A_used=A_com;
%     else     %Moving a Specific Node
%         Lock_Node=zeros(3,3*n_all);
%         Lock_Node(1,Node_Control)=1;
%         Lock_Node(2,Node_Control+n_all)=1;
%         Lock_Node(3,Node_Control+2*n_all)=1;
%         A_used=Lock_Node(1:2,:);
%     end
    
    A=[ A_used; C];  
    b=[ b_used; zeros(size(C,1),1)];

    Loop_Con=[];
    for i=1:N_subsections
        Elements=3;
        Loop_Con=blkdiag(Loop_Con, ones(1,Elements));
    end

%% Compute the required Motion (either centralized or decentralized)

%Do the Bookkeeping for the state vectors.

%Choose whether to use centralized or decentralized control
if Cent_Decentralized_On==1
    [ x_opt, Ldot ] = Controled_Motion_Nodes( y_tot(end,:)', Edges_All, Edges_All_Tube, Loop_Con, L2th, Order, A, b, L_norm, Cost_Fun_Index);
else
    %Compute the decentralized control
    [ x_opt, Ldot ] = Controled_Motion_Nodes_Dist(y_tot(end,:)', Edges_All, Edges_All_Tube, Loop_Con, L2th, Order, A, b, L_norm, Cost_Fun_Index, L_robots, Local);

%     [ x_opt, Ldot ] = Controled_Motion_Nodes_Dist( y_tot(end,:)', Edges_All, Edges_All_Tube, Loop_Con, L2th, Order, A, b, L_norm, Cost_Fun_Index);
end

%From the Desired Edge Motions, extract the Roller Changes
%THis L2th encodes which nodes are passive and which are not. 
Command=L2th*Ldot;  %Need to do this with local information
%It looks like rollers 1 and 2 are the ground rollers.

%Can Potentially simulate unrecognized failure of the rollers
% Command(6)=0;
%Could Potentially turn off the Commands

%Play Back the Commands through the Rollers
[xdot] = Dynamics_Rollers_no_t( y_tot(end,:)', Edges_All, C, Order, B_T, Command );

%% Estimate the next position. 

%Or, propogate the kinematics a certain distance to obtain the target
%position




%Perform the Euler Integration Update (Does this need to account for the
%real time gap?  In practice, I could measure the new state, as opposed to
%compute it through forward propogation. 
dt=toc;
time(count)=dt;
tic
y_tot=[y_tot; xdot'*dt+y_tot(end,:)];
%Could extract some closed loop position information here
t_vec(count+1)=t_vec(end)+dt;
command_hist(count+1,:)=[Command_Normalized];
Node_Control_hist(count+1)=[Node_Control];
%Determine the New Node Positions
Node_Pos_All=[Node_Pos_All; Command'*dt+Node_Pos_All(end,:)];
%
%% Do the plotting if desired, compute the frequency every steps_plot loops
steps_plot=1;
if mod(count,steps_plot)==0
    
    y_current=y_tot(end,:)'; %(count,:);
    y_current_mat=reshape(y_current,n_all,3);
%     Duration=toc;
%     tic

    %Display_Frequency
%     disp(steps_plot/sum(time(end-steps_plot+1:end)))

    
    
    %Do I compute the loads just when doing a display?
    
    %Plot the robot (Shape, Loads, CoM, Edge Labels)\
    %Could get the dynamic Loading Data
%     Indices_Tube=L_Load(:,i); %./Max_Load;
%%
    
%   In practice, make this update the figure axis in the GUI 
        
    Indices_Tube=ones(12,1);
    clf
    
    %Get the Handle from the GUI
    Find_Result=findobj(obj, 'type', 'axes');
%     Find_Result=gcf; %Or could just make my own figure
    
%     hold(Find_Result,'on')
    Plot_Robot_Edges_h(Edges_All(1:N_Edges_True,:),reshape(y_current',n_all,3),Inds_All(1:N_Edges_True), .05, Find_Result);
    hold(Find_Result,'on')
    %     hold on
    
%         Plot_Robot_Edges_Color_Spec( Edges_Test(1:12,:), reshape(y_tot(i,:)',n_all,3), .05, [1 0 0] )
        %Plot the Constraint Connections
    [Edge_Con, x_conPlot]=Plot_Connections( y_current', T2T_Qual );
    Plot_Robot_Edges_h(Edge_Con,x_conPlot,ones(size(T2T_Qual,1),1),.025, Find_Result);
        %Plot the Node/Node Connection Joints with Spheres? 
    Plot_Node_Boxes_h(y_current', T2T_Qual, d_up, d_down, d_length, d_width, Passive, Find_Result);
    
    %Plot the center of mass
    x_com_current=M_Mat*y_tot';
    plot3(Find_Result, x_com_current(1,:),x_com_current(2,:),x_com_current(3,:),'k');
    axis(Find_Result, 'equal');
%     axis(Find_Result, [-2 2 -1 3 0 2.5]); %Set up good axis limits
    
    %Plot the Command in the form of a 3D arrow
    L_arrow=5; %Maximum length of the arrow in the plot 
    
    if Node_Control==2
        Point_Start=x_com_current(:,end);
    else
        Point_Start=y_current_mat(Node_Index,:)';
    end
    
    if XY_Only==0
        Point_End=Point_Start+[Command_Normalized]*L_arrow/Max_Speed_Node;  %How to rescale command length?
    else
        Point_End=Point_Start+[Command_Normalized(1:2); 0]*L_arrow/Max_Speed_Node; 
    end
    
    mArrow3_h(Point_Start, Point_End, Find_Result,'stemWidth',.1,'tipWidth',.15,'color','b','edgecolor','k'); %Plot a 3D Arrow of the Command
    
    %Plot the Center of Mass as a sphere
    [x_sphere,y_sphere,z_sphere]=sphere();
    Size_Sphere=.1;
    surf(Find_Result,Size_Sphere*x_sphere+x_com_current(1,end),Size_Sphere*y_sphere+x_com_current(2,end), Size_Sphere*z_sphere+x_com_current(3,end))
    
    %Number the nodes
    for i=1:n_true
        text(Find_Result, y_current_mat(i,1),y_current_mat(i,2),y_current_mat(i,3),num2str(i),'fontsize',20)
    end
    %To control, need a way to orient the robot with respect to the world
    %frame.  Use one IMU to gather that data? Align the Plot?
    hold(Find_Result,'off')
    drawnow
    
    %% Do the Broadcast over the radio. Does this mess up the comms communication somehow?
    
    if Turn_On_Broadcast==1
       %Need to figure out how to send the position commands over radio
       %Need to have some extrapolation of current position along tube to
       %desired position along the tube.
       
       %Send Position Commands. Do I send these everytime I plot or
       %continuously?  
       Command_sort=[5, 1, 8, 2, 6, 4, 7, 3];  %Vector that relates simulation to actual robot
       Command_Message(Command_sort)=Node_Pos_All(end,:)/in2m; %Send the current roller positions   
    end
    
end

count=count+1;

end

toc

%%

% Some Resorting probably needed to get this to work. 

%Update a display in the GUI with the frequency? 

%% Show all of the plotting information
% plot(t_vec, M_M)
%Add plots for all of the nodes as well to show the motions? 

%% Some Plotting after runnign the simulation

subplot 211  %There may be something weird about where the plotting goes to smooth out the timing.
plot([0,t_vec])
subplot 212
plot(1:size(Node_Pos_All,1),Node_Pos_All')

%% Experiment with sorting and unsorting the commands

%Commands_Data
Sent_Commands=[0  0  0  0 0  0   0  0;
               0 -7 -7 12 0  6   7 -14;
               0 4  4 -9  0 -5  -4 9;
               0 -6 4 -9  0  5   1 4;
               0  0 0  0  0  0   0 0];
           
%Some weirdness here about this. 
Command_sort=[5, 1, 8, 2, 6, 4, 7, 3];  %What does this vector do exactly?

Sent_Commands_Sim=Sent_Commands(2,Command_sort);

%%
Command_Rollers=1:8;

Command_Rollers_Switch=Command_Rollers(Command_sort);

% Here is how to switch the order of these things
test(Command_sort)=Command_Rollers;

test(Command_sort)


%%
% save('Simulation_Run_2')








