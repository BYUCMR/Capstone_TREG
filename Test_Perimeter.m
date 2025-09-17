clc
clear
addpath(genpath(pwd))
%A cleaner (and updated) implementation of estimation for the triangle. 

%% Initialize
%% New Initialization of the robot
in2m=2.54/100;
%Initialize the Robot

% Nodes=[4 6 5; 7 9 8; 12 11 10; 13 15 14; 16 18 17];
d_space=2.5*in2m;  %The spacing between the two rollers
d_offset=10*in2m;   %The distance between the kinematic joint and tube
d_offset_normal=0*in2m; %The Offset to Move Normal to the plane of the tube
d_offset_base=2*in2m;   %Offset of how big the box is. 
% L_tube=134*in2m;
L_tube=(97+108+101.5)/100+3*d_space;  %Measured Lengths
% 97+108+101.5 These are some measured lengths.  It seems like they are a
% bit off
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
n=n_true; %The 
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
% figure
% Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
% hold on
% Number_Nodes( x_all(1:n_true,:))
% axis equal
% title('Nodes')

Lengths=Get_Lengths_E(Edges_All_Tube,x_all);  %Check to make sure that the sizes match.

%Get all of the lengths. This is where the measurements need to be incorporated
Lengths_All=Get_Lengths_E(Edges_All,x_all); 
% 
% sum(Lengths)
% check=(97+108+101.5)/100
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

%Manually Divy up the Edges
Edges_Local{1}=[7 8 4]; %The index of the edges that node 1 reasons about
Edges_Local{2}=[1 5 9 10];
Edges_Local{3}=[3 6 11 12];
Edges_All_Inds=[1 2 3]; %The Edges for the perimeter
%Prepare a Plotting-level Figure


%% Define the constraints to the environment
%5 and 6 are the nodes that probably should be constrained
A_local{1}=zeros(2,2*n_all);
A_local{1}(1,1)=1;
A_local{1}(2,n_all+1)=1;
A_local{2}=zeros(1,2*n_all);
A_local{2}(1,2+n_all)=1; %Constraint in Z
A_local{3}=[];

b_local{1}=zeros(2,1);
b_local{2}=0;
b_local{3}=[];

A_local{3}=A_local{2};
b_local{3}=b_local{2};

A_local_all=[A_local{1}; A_local{2}];
b_local_all=[b_local{1}; b_local{2}];
%% How to divy up the extra edges? 

%Each node know the perimeter, and one edge length.

%Everything in 2D

%Recieve the measured edge lengths
%Not leveraging redundant information in the layout

%Divvy up the cost function

%Each node reasons about the perimeter
L_meas=Lengths_All;
L_meas(1)=1.2;
L_perim=sum(Lengths_All(1:3));
for i=1:n
%     Inds_Rel=find(Edges_All(:,1)==i); %Find the incident edges
%     Cost{i}= @(x) Cost_Dist(x, Edges_All(Inds_Rel,:), L_noise(Inds_Rel));
    Cost{i}= @(x) Cost_Dist(x, Edges_All(Edges_Local{i},:), Edges_All(Edges_All_Inds,:), L_meas(Edges_Local{i}), L_perim, Order(i,:))
end
x_start=reshape(x_all(:,1:2),2*n_all,1); %Reshape into a vector form
options = optimoptions('fmincon','Display','iter','SpecifyObjectiveGradient',true,'CheckGradient',false)
for i=1:3
   x_opt_cent=fmincon(Cost{i}, x_start+.2*rand(size(x_start)),[],[],A_local_all,b_local_all,[],[],[],options)
end


%
Cost_All=@(x) Cost{1}(x)+Cost{2}(x)+Cost{3}(x);
Cost_All(x_start+rand(size(x_start)))
%Need to check derivatives on this....
options = optimoptions('fmincon','Display','iter','SpecifyObjectiveGradient',false,'CheckGradient',false)

% options = optimoptions('fmincon','Display','iter',SpecifyObjectiveGradient',true,'CheckGradient',true)
%Need to add the additional constraints here....
% x_opt_cent=fminunc(Cost_All, x_start+rand(size(x_start)),options)
x_opt_cent=fmincon(Cost_All, x_start+.2*rand(size(x_start)),[],[],A_local_all,b_local_all,[],[],[],options)

figure
Plot_Edges(Edges_All, reshape(x_opt_cent,n_all,2),'o-')  %All of the Required Edges
hold on
Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
axis equal
%


Lengths_opt=Get_Lengths_E(Edges_All,reshape(x_opt_cent,n_all,2)); 
max(abs(x_opt_cent-x_start))

%%
%Can update to speed up the exit function by updating matlab, as this was fixed in R2018a

%Set the options only once
%Perform the ADMM Updates
% tic

K = 25; % number of timesteps
c_p = .1; % weight for agreement
c_r = 1; % weight for constraint
%Initialize the different variables for faster computation
N_robots=n
n_states=2*n_all;
p = zeros(n_states, N_robots, K+1); % agreement dual
for i=1:N_robots
    r{i} = zeros(size(A_local{i},1), K+1); % constraint dual
end
x_ADMM=zeros(n_states, N_robots, K+1); %Decision Variable
x_ADMM(:,:,1)=repmat(x_start+.1*rand(size(x_start)),1,N_robots); %Injecting noise into this initial guess
% rng(1); %Seed the randomness
% Var=2;
% x(:,:,1)=repmat(reshape(x_all,3*n_all,1),1,N_robots)+[normrnd(0,Var,n_all*2,N_robots); zeros(n_all,N_robots)];
Adj=ones(3)-eye(3);
L=diag(sum(Adj))-Adj;
absL=abs(L);
for i=1:N_robots
    Neighbors{i}=find(Adj(i,:));
end
%%Gradient does not appear to be correct...
tic
options = optimoptions('fminunc','Display','off','SpecifyObjectiveGradient',true,'CheckGradient',false,'Algorithm','quasi-newton','UseParallel',false,'OptimalityTolerance',1e-5,'HessUpdate','bfgs'); %'quasi-newton'

for k = 1:K
    for i = 1:N_robots
        p(:, i, k+1) = p(:, i, k) + c_p * x_ADMM(:, :, k) * L(i, :)'; %Cool computation here
        r{i}(:, k+1) = r{i}(:, k) + c_r * (A_local{i}*x_ADMM(:, i, k) - b_local{i}); %Duals for local constraint
%         Obj=@(x_var) Obj_Attempt( x_var, i, D, p, r{i}, c_p, c_r, Neighbors, k, x, m, xdes );
%         Obj(x(:,i,k))
        %Note that no gradient information is used here.  Could that
        %improve performance?
                %         Obj2=@(x_var) Obj_Augmented_L( x_var, Cost{i}, i, p(:,i,k+1), r{i}(:,k+1), c_p, c_r, Neighbors, x(:,:,k), A_local{i}, b_local{i});
        Obj3=@(x_var) Obj_Augmented_L_grad2( x_var, Cost{i}, i, p(:,i,k+1), r{i}(:,k+1), c_p, c_r, Neighbors, x_ADMM(:,:,k), A_local{i}, b_local{i}, absL);
        [x_test, val, flag]=fminunc(Obj3,x_ADMM(:,i,k),options);
        x_ADMM(:,i,k+1)=x_test;
%         val1(i,k)=Obj2(x_test);
%         val_all(i,k)=Cost_All(x_test);
%         disp(flag)
    end
    
end
Duration_solve=toc
%
figure
subplot 311
color = jet(n);
for i = 1:N_robots
    for j = 1:n
    plot(0:K, permute(x_ADMM(j,i,:), [1 3 2]), 'Color', color(j, :),'Linewidth',.5)
    hold on
    end
end
xlabel('Iteration')
ylabel('Velocity')

subplot 312

%Need to get the constraints plotted here!
for i=1:N_robots
%     for j = 1:n
        x_i=permute(x_ADMM(:,i,:), [1 3 2]);
        semilogy(0:K, abs(A_local{i}*x_i-b_local{i}), 'Color', color(j, :),'Linewidth',.5)
        hold on
%     end
end
xlabel('Iteration')
ylabel('Constraint Violation')

subplot 313
clear Cost_I
for i=1:K
    for j=1:N_robots
        Cost_I(i,j)=Cost_All(x_ADMM(:,j,i));  %Obj is the centralized objective from above
    end
end
semilogy(1:K,Cost_I)

xlabel('Iteration')
ylabel('Centralized Cost')


%% 
x_results=x_ADMM(:,1,end)
        
     
figure
Plot_Edges(Edges_All, reshape(x_results,n_all,2),'o-')  %All of the Required Edges
hold on
Plot_Edges(Edges_All, x_all,'*--')  %All of the Required Edges
axis equal

Lengths_opt2=Get_Lengths_E(Edges_All,reshape(x_results,n_all,2))
[Lengths_opt2, Lengths_opt, Lengths_All]