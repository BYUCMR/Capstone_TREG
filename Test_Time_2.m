tic
options = optimoptions('fminunc','Display','off','SpecifyObjectiveGradient',true,'CheckGradient',false,'Algorithm','quasi-newton');
K=40;
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