% initialize the model using spacial-v2
model = model_biped_5link();
% q = [x, z, theta, q1_r, q2_r, q1_l, q2_l]

q  = [0; 0.4; 0; pi/3; -2*pi/3; pi/3; -2*pi/3];
% q  =  [0; 0.8; 0; 0; 0; 0; 0];



t_data = [0, 1];
showmotion(model, t_data, [q, q])

[H, C] = HandC(model, q, dq);

H_sym = H_5link(q);

% FK_rf(q)
% FK_lf(q)

% unit testing inertia matrix and bias terms
N = 100;
for i = 1:N
    q = rand(7, 1); % needs to be a column vector
    dq = rand(7, 1);

    % compute mass matrix H and bias term C using spatial-v2
    [H, C] = HandC(model, q, dq);
    
    % compute mass matrix H_sym and bias term bias_sym symbolically
    H_sym = H_5link(q);
    bias_sym = bias_5link(q, dq);

    diff_H = H - H_sym;
    diff_bias = C - bias_sym;

    assert(norm(diff_H) < 1e-5);
    assert(norm(diff_bias) < 1e-5);
end

% forward kinematics of the contact foot

q = [0; 0; 0; pi/2; 0; 0; 0];
dq = [0; 0; 0; 0; 0; 0; 0];
p_rf = FK_rf(q);
p_lf = FK_lf(q);

% % visualization
t_data = [0, 1];
showmotion(model, t_data, [q, q])
 
% forward dynamics using spatial-v2
% tau_zero = [0; 0; 0; 0; 0];
% ddq = FDab(model, q, dq, tau_zero); % in our contact model, we dont use this function, we will use contact constrained dynamics instead
% 
N = 100;
for i = 1:N
    q = rand(7, 1); % needs to be a column vector
    dq = rand(7, 1);
    tau = zeros(7, 1);
    tau(4:7) = rand(4, 1);

    % forward dynamics using spatial-v2
    ddq = FDab(model, q, dq, tau);

    % forward dynamics using symbolic expressions
    % compute mass matrix H and bias term C using spatial-v2
    H_sym = H_5link(q);
    bias_sym = bias_5link(q, dq);
    ddq_sym = H_sym \ (tau - bias_sym);

    diff = ddq_sym - ddq;
    assert(norm(diff) < 1e-5);
end
% 
% contact constraint forward dynamics
q = [0; 0; 0; 0; 0; 0; 0];
dq = [0; 0; 0; 0; 0; 0; 0];
tau = [0; 0; 0; 0; 0; 0; 0];

Jc_rf = dFK_rf_dq(q);
Jc_dot_rf = Jc_dot_rf_func(q, dq);
H_sym = H_5link(q);
bias_sym = bias_5link(q, dq);

A = [H_sym, -Jc_rf'; Jc_rf, zeros(2)];
b = [tau - bias_sym; -Jc_dot_rf * dq];

x = A\b;
ddq = x(1:7);
lambda = x(8:9);