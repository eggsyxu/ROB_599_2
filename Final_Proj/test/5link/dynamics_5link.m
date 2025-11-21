function [dX, lambda] = dynamics_5link(X, U)
    q  = X(1:7);
    dq = X(8:14);

    B = zeros(7,4);
    B(4:7,:) = eye(4);

    H     = H_5link(q);         
    bias  = bias_5link(q, dq);  

    J_rf = dFK_rf_dq(q);       
    J_lf = dFK_lf_dq(q);      
    Jc   = [J_rf; J_lf];        

    J_rf_dot = Jc_dot_rf_func(q, dq);
    J_lf_dot = Jc_dot_lf_func(q, dq);
    Jc_dot   = [J_rf_dot; J_lf_dot]; 

    A = [H, -Jc'; Jc, zeros(4)];
    b = [B*U - bias; -Jc_dot*dq];

    x = A \ b;
    ddq    = x(1:7);
    lambda = x(8:11);
   
    dX = [dq; ddq];
end
