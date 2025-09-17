function [ Output, grad ] = Obj_Augmented_L_grad( x_var, Obj, i, p, r, c_p, c_r, Neighbors, x, Aeq, beq, absL)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

Sum=0;
    
for j=1:length(Neighbors{i})
   Sum=Sum+norm(x_var-(x(:,i)+x(:,Neighbors{i}(j)))./2)^2;
%    Sum=Sum+norm(x_var-(x(:,i,k)+x(:,Neighbors{i}(j),k))./2)^2;
end
% Obj{i}(x_var)
[Obj_Val, Obj_grad]=Obj(x_var);
Output= Obj_Val + p'*x_var + c_p*(Sum) + r'*(Aeq*x_var-beq) + c_r*norm(Aeq*x_var-beq)^2;
% Output= Obj{i}(x_var) + p(:,i,k+1)'*x_var + c_p*(Sum) + r(i,k+1)*(m'*x_var-xdes) + c_r*norm(m'*x_var-xdes)^2;

% Output= .5*x_var'*D(i,:)'*D(i,:)*x_var+p(:,i,k+1)'*x_var+c_p*(Sum)+r(i,k+1)*(m'*x_var-xdes)+c_r*norm(m'*x_var-xdes)^2;

%The gradient of the augmented Lagrangian
grad=Obj_grad'+ p + Aeq'*r + - 2*c_r*Aeq'*beq + 2*c_r*(Aeq'*Aeq)*x_var - c_p * x * absL(i, :)' + 2*c_p*absL(i, i)*eye(length(x_var))*x_var ; %Compute the gradient of the objective


% Output=Obj_Val;
% grad=Obj_grad;

% Output= Cost + p(:,i,k+1)'*x_var+c_p*(Sum)+r(i,k+1)*(m'*x_var-xdes)+c_r*norm(m'*x_var-xdes)^2;
% grad=dCost+p(:,i,k+1) - c_p * x(:, :, k) * absL(i, :)'  + r(i, k+1) * m - 2*c_r*m*xdes + 2*c_r*(m*m')*x_var+ 2*c_p*absL(i, i)*eye(n)*x_var; 



%Don't even need to apply the gradient, but I could potentially use the
%gradient to speed up the computation


end

