function initialize_optimization()
    syms x1 x2 real
    x = [x1; x2];

    % 一个凸的目标函数
    f = 0.5*x1^2 + 2*x2^2 + x1*x2 + 2*x1;

    grad = gradient(f, x);
    Hess = hessian(f, x);

    matlabFunction(f, grad, Hess, 'File', 'fcn_f_grad_Hess', 'Vars', {x1, x2});
end
