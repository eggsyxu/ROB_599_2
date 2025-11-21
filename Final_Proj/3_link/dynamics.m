function [dX, lambda] = dynamics(X, U)
    q = X(1:5);
    dq = X(6:10);
    % construct selection matrix
    B = zeros(5, 2);
    B(4, 1) = 1;
    B(5, 2) = 1;

    Jc = dFK_dq_foot(q);
    Jc_dot = Jc_dot_func(q, dq);
    H_sym = H_3link(q);
    bias_sym = bias_3link(q, dq);
    
    A = [H_sym, -Jc'; Jc, zeros(2)];

    b = [B * U - bias_sym; -Jc_dot * dq];
    
    x = A\b;
    ddq = x(1:5);
    lambda = x(6:7);
    dX = [dq; ddq];
end