% initialize the model using spacial-v2
model = model_biped_3link();

% q = [0 - 0.039960169001852; 0.8 - 0.496472381958992; -pi/6; pi/2; -3 * pi/4];
q = [0 ; 0.8 - 0.006844110900952 ; -pi/24; pi/12; -pi/12];
t_data = [0, 1];
showmotion(model, t_data, [q, q])

pf = FK_foot(q);

% unit testing inertia matrix and bias terms
N = 100;
for i = 1:N
    q = rand(5, 1); % needs to be a column vector
    dq = rand(5, 1);

    % compute mass matrix H and bias term C using spatial-v2
    [H, C] = HandC(model, q, dq);
    
    % compute mass matrix H_sym and bias term bias_sym symbolically
    H_sym = H_3link(q);
    bias_sym = bias_3link(q, dq);

    diff_H = H - H_sym;
    diff_bias = C - bias_sym;

    assert(norm(diff_H) < 1e-5);
    assert(norm(diff_bias) < 1e-5);
end

% forward kinematics of the contact foot
pf = FK_foot(q);

% visualization
t_data = [0, 1];
showmotion(model, t_data, [q, q])

% forward dynamics using spatial-v2
tau_zero = [0; 0; 0; 0; 0];
ddq = FDab(model, q, dq, tau_zero); % in our contact model, we dont use this function, we will use contact constrained dynamics instead

N = 100;
for i = 1:N
    q = rand(5, 1); % needs to be a column vector
    dq = rand(5, 1);
    tau = zeros(5, 1);
    tau(4:5) = rand(2, 1);

    % forward dynamics using spatial-v2
    ddq = FDab(model, q, dq, tau);

    % forward dynamics using symbolic expressions
    % compute mass matrix H and bias term C using spatial-v2
    H_sym = H_3link(q);
    bias_sym = bias_3link(q, dq);
    ddq_sym = H_sym \ (tau - bias_sym);
    
    diff = ddq_sym - ddq;
    assert(norm(diff) < 1e-5);
end

% contact constraint forward dynamics
q = [0; 0; 0; 0; 0];
dq = [0; 0; 0; 0; 0];
tau = [0; 0; 0; 0; 0];

Jc = dFK_dq_foot(q);
Jc_dot = Jc_dot_func(q, dq);
H_sym = H_3link(q);
bias_sym = bias_3link(q, dq);

A = [H_sym, -Jc'; Jc, zeros(2)];
b = [tau - bias_sym; -Jc_dot * dq];

x = A\b;
ddq = x(1:5);
lambda = x(6:7);