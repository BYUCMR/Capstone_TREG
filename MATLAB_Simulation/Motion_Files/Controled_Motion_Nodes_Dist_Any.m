function [ x_opt, Ldot ] = Controled_Motion_Nodes_Dist_Any( x, Edges_All, Edge_All_Tube, Loop_Con, L2th, Order, A, b, L_norm, Cost_Index, L, Local, Assign_Passive )
%Take in a desired motion. It will sovle for a velocity of each node
% that minimizes roller effort while satisfying all constraints

%This expects local as a cell array, not a matrix

%Note, the incoming A specifies the ground points 

%The incoming A specifies the motion of all of the specified nodes

%Note! I need to import seperate Edge Lists for the Tubes and for the
%constraints
N_True=size(Edge_All_Tube,1);
n=length(x)/3;
R_tot=Get_Rtot(x, n, Edges_All, Order);
R=R_tot(1:N_True,:);
R_con=R_tot(N_True+1:end,:);  %The constraint portions of the matrix
b_con=zeros(size(R_con,1),1);
%The constraint on the constant volume between true lengths
A_LoopCon=Loop_Con*R;
b_LoopCon=zeros(size(Loop_Con,1),1);

Aeq=[R_con; A; A_LoopCon]; 
beq=[b_con; b;    b_LoopCon];

%% Compute the necessary "divisions," how things are seperatley located
N_robots=max(Order(:,1)); %The number of kinematic nodes, or actual robots. (Hardcoded for now)


% %%  Compute the Rigidity Matrices
% 
% %(Currently done in a centralized fashion, but they only rely on relative
% %positions between neighbors)
% 
% [R_Edges]=Rigidity_Matrix_Edges(Edges_Tube,reshape(x,n,3));
% [R_Edges_Con]=Rigidity_Matrix_Edges(Edges_Con,reshape(x,n,3));
%  
% % [Angles, Grad]=Check_Angle_Constraints( Angles_Con, x_vec, Num_Steps ); %This is wrong somehow.
% % R_Angle=Grad(1:2*n,:)';
% 
% % R_Angle=Grad';
% %Also Need to Generate Order, the planarity joint.
% [ ~, Grad ] = Planartiy_Con( x_vec, Order(:,[ 2, 3, 4, 1]));
% R_Planar=Grad';   %This seems right!s
% 
% [ ~, Grad2 ] = Planartiy_Con( x_vec, Order(:,[4, 5, 2, 1]));
% R_Planar2=Grad2'; 
% % Get the Bisection Constraints
% % [~, Grad_2]=Bisection_Constraint( x_vec, Order );
% % [~, Grad_Angle]=Equal_Angle_Constraint_Shift(x_vec, Order);
% [~, Grad_Angle] = Planar_Bisection_Constraint( x_vec, Order );
% % R_Bisect=Grad_2';
% R_Bisect=Grad_Angle';
% R_tot=[R_Edges; R_Bisect; R_Planar; R_Planar2]; %Generate the total planarity joint

%% Pick an Objective

%Determine how to do the same thing with distributed control

switch Cost_Index
    
    case 1
        Obj=[R];  %This objective is minimizing change in edge length, not necessarily roller.
        % Obj=[L2th*R]; %Minimize the Motion of the Roller Nodes
        f=[];
        H=2*(Obj'*Obj);  %I think I need factor fo two to get scaling right.

    case 2
        %Formation Objective
        %Compute the Edge Lengths. 
        Lengths=Get_Lengths_E(Edge_All_Tube,reshape(x,n,3));
        L_norm_vec=Lengths; %Include other edges as constrained?
        L_norm_vec(1:size(Edge_All_Tube,1))=L_norm;
        dx=((Lengths-L_norm_vec)'*R)';  %Note that this is only for the tube ends.  I think that makes sense

        Mat=blkdiag(zeros(6),eye(n-6));
        H=blkdiag(Mat,Mat,Mat);
        f=dx;

    case 3  %Minimum Roller Motion?
        
end

%% Divide up the centralized problem into the distributed problem. 

%Note that this assumes that it is divisible.  Even if it is not.  It will
%divide it, even if it includes some non-divisible parts. 

for i=1:N_robots
    A_split{i}=[];
    b_split{i}=[];
    Obj_Local{i}=[];
end

%This could be moved to only run with changes in the loop, but probably
%fine like this. 

%This method is essentially ensuring that each row goes with one robot
for i=1:size(Aeq,1)-size(A_LoopCon,1)
    Inds=find(Aeq(i,:)); %If any of these indices are active...
    for j=1:N_robots  
        count(j,i)=numel(intersect(Inds,[Local{j},Local{j}+n,Local{j}+2*n]));
    end
    
    %Add each row to the node with the maximum
    [~,Node_Assign]=max(count(:,i));
    Node_Assign=Node_Assign(1); %Take the first if there are multiple
    A_split{Node_Assign}=[ A_split{Node_Assign}; Aeq(i,:)];
    b_split{Node_Assign}=[b_split{Node_Assign}; beq(i)];

end

%Assign the constant length constraint to each passive node
% Assign_CL is a vector the same length as A_LoopCon that makes assignments
for i=1:size(A_LoopCon,1)
    A_split{Assign_Passive(i)}= [A_split{Assign_Passive(i)};  A_LoopCon(i,:)];
    b_split{Assign_Passive(i)}=[b_split{Assign_Passive(i)}; 0];
end

% % (For now hardcoded)
% %Need to be able to divy up this constraint automatically!
% Passive=[1,3];
% %Assign the constant length constraints to the passive nodes
% A_split{Passive(1)}= [A_split{Passive(1)};  A_LoopCon([1,4],:)];
% b_split{Passive(1)}=[b_split{Passive(1)}; [0; 0]];
% A_split{Passive(2)}= [A_split{Passive(2)};  A_LoopCon([2,3],:)];
% b_split{Passive(2)}=[b_split{Passive(2)}; [0; 0]];

%Now divy up the cost function
for i=1:N_robots
    for j=1:size(Obj,1)
        Inds=find(Obj(j,:)); %If any of these indices are active...
        if ~isempty(intersect(Inds,[Local{i},Local{i}+n,Local{i}+2*n]))
            Obj_Local{i}=[ Obj_Local{i}; Obj(j,:)];
        end
    end
end

%% Compute the Inverses

D_Ind=Obj_Local;
A_local=A_split;
b_local=b_split;

K = 1000; % number of timesteps
c_p = .1; % weight for agreement
c_r = 1; % weight for constraint

%Prepare the parameters of the optimization
n_states=3*n;
p = zeros(n_states, N_robots, K+1); % agreement dual
for i=1:N_robots
    r{i} = zeros(size(A_local{i},1), K+1); % constraint dual
end
x = zeros(n_states, N_robots, K+1); %Decision Variable

% L = diag(ones(n-1, 1), 1) + diag(ones(n-1, 1), -1); % graph Laplacian
% L = diag(sum(L,2)) - L;
% L=diag(sum(Adj))-Adj;
% J = zeros(n_nodes, K+1); % central cost
for i = 1:N_robots
    Mat(:, :, i) = (D_Ind{i})'*(D_Ind{i}) + 2*c_r*(A_local{i}'*A_local{i}) + 2*c_p*L(i, i)*eye(n_states);
%     Mat(:, :, i) = (Mask{i}*D)'*(Mask{i}*D) + 2*c_r*(A'*A) + 2*c_p*L(i, i)*eye(n);
%     Mat(:, :, i) = D(i, :)'*D(i, :) + 2*c_r*(A'*A) + 2*c_p*L(i, i)*eye(n);
    invM(:, :, i) = inv(Mat(:, :, i));
    x(:, i, 1) = invM(:, :, i)*zeros(n_states, 1);
end

%% Iterativley Determine the Result

absL = abs(L);
% tic
% absL=Adj;
for k = 1:K
    for i = 1:N_robots
        p(:, i, k+1) = p(:, i, k) + c_p * x(:, :, k) * L(i, :)'; %Cool computation here
        r{i}(:, k+1) = r{i}(:, k) + c_r * (A_local{i}*x(:, i, k) - b_local{i}); %Duals for local constraint
        x(:, i, k+1) = invM(:, :, i) * (2 * c_r * A_local{i}'*b_local{i} ...
            - p(:, i, k+1) ...
            - A_local{i}'*r{i}(:, k+1)  ...
            +c_p * x(:, :, k) * absL(i, :)'); %The linear update
%         J(i, k+1) = x(:, i, k+1)'*(D'*D)*x(:, i, k+1); %Compute Centralized Cost at each node
    end
end
x_dist=x(:,1,end-1); %Take one of the last columns to apply the control. 
x_end=x(:,:,end-1);
%Check for termination. Take the difference between random rows? 
if max(abs(x_end(:,1)-x_end(:,2)))>1e-3
    x_dist=zeros(length(x_dist),1);
    disp('Warning, Bad Solve')
end
    

%%
% % Maintain equal Edge Lengths
% L=Get_Lengths_E(Edge_All_Tube, reshape(x,n,3)); %Compute the Length Vector
% L_norm=mean(L(1:3))*ones(size(Edge_All_Tube,1),1);
% dx_desired=(2*(L'-L_norm')*R)'; %How do I wan to manage this?
% Obj=eye(length(x));
% f=dx_desired;
% H=Obj;

%in some cases I could just solve with the matrix inverse

% %% Head Towards a Targt
% %This guy is messing things up if I only want to control part of the CoM,
% %obviously. 
% enable_Waypoints=true;
% if enable_Waypoints
%     b_des=(target-A(1:3,:)*x)/norm(target-A(1:3,:)*x)*speed;  %This only really works for CoM, not for other points
%     b(1:3)=b_des(1:3);
% end
%Probably need to do one of these for each point? If we want to match speed
%at least.
%%
%Augment with Angle Constraints
% Aeq=[R_con; A_LoopCon; A]; 
% beq=[b_con;     b_LoopCon; b];
% 
% %Could solve using a matrix inverse...
% % [x_opt]=inv([H'*H, Aeq'; Aeq, zeros(size(Aeq,1),size(Aeq',2))])*[zeros(size(H,1),1); beq];
% % options =  optimoptions('Display','off');
% % x0=Aeq\(-beq);
% options = optimset('Display', 'off','MaxIter',10000); %'algorithm','trust-region-reflective');
% x0=zeros(length(x),1);
% [x_opt,fval,exitflag,output]=quadprog(H,f,[],[],Aeq, beq,[],[],x0,options); %Solve the quadratic program
% % It seems like with only equality constraints this should be solved
% % directly by forming the augmented lagrangian and finding the inverse
% % [x_lambda(1:length(x_opt)), x_opt]
% if ~(exitflag==1)
%     output.message
%     disp('Warning, Bad Solve')
%     x_opt=zeros(length(x0),1); %If unable to solve, simply return 0s for the desired velocities.
% end
% 
% max(abs(x_opt-x_dist))


x_opt=x_dist;
All_Inputs=R_tot*x_opt;
Ldot=All_Inputs(1:size(Edge_All_Tube,1)); %Isolate the commands that apply to the edges

end

