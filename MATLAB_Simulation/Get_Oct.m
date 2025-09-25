function [ xI, xF, Adj ] = Get_Oct(  )
%Generate an Octahedron

%Line up an Octahedron Appropriatley...

%Start with Face 1 on the Ground...
a=1;
b=sqrt(2)/2;
x=[b -b 0;
   b b  0;
   -b b 0;
   -b -b 0;
   0  0  -a
   0  0  a];

x(:,1)=x(:,1)-b
x=x./sqrt(2);
Adj=[0 1 0 1 1 1;
     0 0 1 0 1 1;
     0 0 0 1 1 1;
     0 0 0 0 1 1;
     0 0 0 0 0 0;
     0 0 0 0 0 0];
Adj=Adj+Adj'; %Make this Symetric

G=graph(Adj);
N_Edges=size(G.Edges{:,1},1);
n=size(x,1);
Edges=G.Edges{:,1}

n=size(x,1);
N_L=size(Edges,1);

% Hackgplot(Adj,x)

%Rotate About the Initial 
Indices=[4 1 4 1 2 4 2 3 3 2 3 1];
% Plot_Edges(Edges,x,Indices)
L=Get_Lengths_E(Edges,x)
% Get_Lengths(Edges,x) %Make sure everything starts well

th=pi-atan2(x(5,3),x(5,1))

Rot=[cos(th)  0   sin(th);
      0     1       0;
      -sin(th)  0   cos(th)]
  
%Rotate everything about this matrix
x=(Rot'*x')'
th=30*pi/180;
% Hackgplot(Adj,x)
Rotz=[cos(th)  sin(th)      0    ; 
      -sin(th) cos(th)      0;
      0         0           1];
xfinal=(Rotz'*x')';
%% Get the Initial Configuration
xfinal(:,1)=xfinal(:,1)-xfinal(5,1)
xfinal(:,2)=xfinal(:,2)-xfinal(5,2)
xfinal(:,3)=xfinal(:,3)-xfinal(5,3)
xI=xfinal;
%%
% Hackgplot(Adj,xfinal)
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')

%% Get the Final Configuration
% xI=x;
% xF=x;

%Center at Vertex 1, Rotate about Z, Rotate about Y, Rotate Back About Z
x_temp=xfinal;
xM=x_temp(1,1);
yM=x_temp(1,2);
zM=x_temp(1,3);
x_temp(:,1)=x_temp(:,1)-xM;
x_temp(:,2)=x_temp(:,2)-yM;
x_temp(:,3)=x_temp(:,3)-zM;
x_temp=(Rotz*x_temp')';
% Hackgplot(Adj,x)

th=atan2(x_temp(6,3),x_temp(6,1))
%Rotate about y
Rot=[cos(th)  0   sin(th);
      0     1       0;
      -sin(th)  0   cos(th)]
%Do the Roll Over...
x_temp=(Rot*x_temp')';
%Flip Back
% Hackgplot(Adj,x_temp)
x_temp=(Rotz'*x_temp')';
x_temp(:,1)=x_temp(:,1)+xM;
x_temp(:,2)=x_temp(:,2)+yM;
x_temp(:,3)=x_temp(:,3)+zM;
xF=x_temp;
%Do Some Basic Centering and Such
% Hackgplot(Adj,x_temp)
% axis equal
% hold on 
% Hackgplot(Adj,xI)
% 
% for i=1:n
%     Vec=[xI(i,:); x_temp(i,:)];
%     plot3(Vec(:,1),Vec(:,2),Vec(:,3))
% end


end

