function [out_J, out_grad] = Cost_Dist( x, Edges, Edges_All, L_meas, L_tot, Order)


%Do the Bisection
%Include the edge optimization
d=2; %Note that I am hardcoding for two-dimensional robots
n=length(x)/d;
% x_hat=reshape(x,d,n)';
x_hat=reshape(x,n,d);
m=size(Edges_All,1);
x_hat=[x_hat, zeros(n,1)];
%Differing convention on thse things...

%What to do about different dimensinoality?  For now just put it back in R3
d=3; 
%% Perimeter Constraint  
% NOTE:  THIS USES A DIFFERENT CONVENTION FOR SORTING THE STATE VECTOR
L_tube=Get_Lengths_E(Edges_All,x_hat);
J_P=(sum(L_tube)-L_tot).^2;
[R]=Rigidity_Matrix_Edges(Edges_All,reshape(x_hat,n,d));
grad_P=2*(sum(L_tube)-L_tot)*(R'*ones(m,1));

% mat_form=reshape(grad,n,d);
% grad_P=reshape(mat_form',d*n,1);

%% Bisection Constraints
[dif_Bisection, Grad_Angle] = Planar_Bisection_Constraint( reshape(x_hat, d*n,1), Order );
% R_Bisect=Grad_2';
J_bisect=dif_Bisection'*dif_Bisection;
grad_bisect=2*dif_Bisection'*(Grad_Angle');

%% Edge Constraints
%Only uses the partial edges list (No special weighting for distances)
L=Get_Lengths_E(Edges,x_hat);
J_L=(L-L_meas)'*(L-L_meas);
[R]=Rigidity_Matrix_Edges(Edges,x_hat);
grad_L=(2*L' - 2*L_meas')*R;
%Use of edge lengths only
% out_J=J_L;
% out_grad=grad_L;
% out_J=J_L+J_bisect;
% out_grad=grad_L+grad_bisect;

%Turn off the bisection constraint for the moment
% out_J=J_L;
% out_grad=grad_L;

% out_J=J_P;
% grad_all=grad_P;
out_J=J_bisect+J_L+J_P;
grad_all=grad_bisect'+grad_L'+grad_P;
out_grad=grad_all(1:2*n); %Export only the 2D portion of this

end

