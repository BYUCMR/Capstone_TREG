absL=abs(L);
options = optimoptions('fminunc','Display','off','SpecifyObjectiveGradient',true,'CheckGradient',false,'Algorithm','quasi-newton');
tic
for k = 1:K
    for i = 1:N_robots
        p(:, i, k+1) = p(:, i, k) + c_p * x(:, :, k) * L(i, :)'; %Cool computation here
        r{i}(:, k+1) = r{i}(:, k) + c_r * (A_local{i}*x(:, i, k) - b_local{i}); %Duals for local constraint
%         Obj=@(x_var) Obj_Attempt( x_var, i, D, p, r{i}, c_p, c_r, Neighbors, k, x, m, xdes );
%         Obj(x(:,i,k))
        %Note that no gradient information is used here.  Could that
        %improve performance?
        
%         Obj2=@(x_var) Obj_Augmented_L( x_var, Cost{i}, i, p(:,i,k+1), r{i}(:,k+1), c_p, c_r, Neighbors, x(:,:,k), A_local{i}, b_local{i});
        Obj3=@(x_var) Obj_Augmented_L_grad( x_var, Cost{i}, i, p(:,i,k+1), r{i}(:,k+1), c_p, c_r, Neighbors, x(:,:,k), A_local{i}, b_local{i}, absL);
        [x_test, val, flag]=fminunc(Obj3,x(:,i,k),options);
        x(:,i,k+1)=x_test;
%         val1(i,k)=Obj2(x_test);
        val2(i,k)=Obj3(x_test);
        % Some debugging things
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