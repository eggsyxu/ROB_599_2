% Path
run('E:\OneDrive\Desktop\study\Course\ROB_599_2\codeBase\setup.m');  % Replace your own path
import casadi.*
addpath(genpath(pwd));

% time step parameters
N = 30;                % number of collocation points
ds = 1/N;              % normalized step size

% initial and final states
% q = [x, y, theta, q1_r, q2_r, q1_l, q2_l]
% q0 = [0 ; 0.8 - 0.107179676972449; -pi/6; pi/3; -pi/3; pi/3; -pi/3]
% % q0 = [0 ; 0.8; -(pi); pi ; 0; pi ; 0];
% q0 = [0 ; 0.8; -pi/2; pi/2; 0; pi/2; 0];
q0  = [0 ; 0.8 - 0.006844110900952; -pi/24; pi/12; -pi/12; pi/12; -pi/12];
% q0_guess = [0; 0.8; -pi/24; pi/12; -pi/12; pi/12; -pi/12];
% prf = full(FK_rf(q0_guess));
% q0_guess(2) = q0_guess(2) - prf(2);  
% q0 = q0_guess;

dq0 = zeros(7,1);
X0  = [q0; dq0];

qf = [0 ; 0.8 - 0.107179676972449; -pi/6; pi/3; -pi/3; pi/3; -pi/3]; 
dqf = zeros(7,1);
Xf  = [qf; dqf];

tic;

% optimization variables
opti = casadi.Opti();
X  = opti.variable(14, N+1);   % [q; dq]
U  = opti.variable(4,  N+1);   % 4 joint torques
tf = opti.variable(1,1);
dt = tf * ds;

obj = MX(0);

% TO function parameters
final_time = 5;
cost_idx = 0;
case0_k = 0.1;
xmax = [0.02, 0.02, deg2rad(3), deg2rad(5), deg2rad(5), deg2rad(5), deg2rad(5), ...
        0.2,  0.2,  deg2rad(20), 0.5, 0.5, 0.5, 0.5];  
Q = diag(1./(xmax.^2));
umax = [120 120 120 120]; 
R = diag(1./(umax.^2));   
R = R * 5;               

% initial constrain
opti.subject_to(X(1:7, 1) == q0);
dq0 = X(8:14,1);
J_rf0 = dFK_rf_dq(dq0);
J_lf0 = dFK_lf_dq(dq0);
v_rf = J_rf0 * dq0;
v_lf = J_lf0 * dq0;
opti.subject_to(v_rf == [0; 0]);
opti.subject_to(v_lf == [0; 0]);

opti.subject_to(tf >= 1);
opti.subject_to(tf <= 3);

% initial guess
opti.set_initial(tf, 1);
opti.set_initial(X, repmat(X0,1,N+1));
opti.set_initial(U, zeros(4,N+1));


for k = 1:N
    Xk   = X(:,k);
    Uk   = U(:,k);
    Xkp1 = X(:,k+1);
    Ukp1 = U(:,k+1);
    p_lf = FK_lf(Xk(1:7));
    p_rf = FK_rf(Xk(1:7));
    % err = 1e-4;
    % opti.subject_to(abs(p_rf(2)) <= err);
    % opti.subject_to(abs(p_lf(2)) <= err);
    opti.subject_to(p_rf(2) == 0);
    opti.subject_to(p_lf(2) == 0);


    % cost function
    switch cost_idx
        case 0
                obj = obj + ...
                case0_k * ((Uk'*Uk)) + ...
                case0_k * ((Ukp1-Uk)'*(Ukp1-Uk)) + ...
                case0_k * (p_lf(1)*p_lf(1)) + ...
                case0_k * (p_rf(1)*p_rf(1));
            % obj = obj + ... 
            %     case0_k * ((Uk'*Uk)) + ...
            %     case0_k * ((Ukp1-Uk)'*(Ukp1-Uk));

        case 1  
            obj = obj + ...
                Xk' * Q * Xk + Uk' * R * Uk;  % Implementing Regularized Predictive Control for Simultaneous Real-Time Footstep and Ground Reaction Force Optimization

        otherwise
            error('Invalid cost_idx value.');
    end

    % dynamics constrains
    [fk,   fi] = dynamics_5link(Xk,   Uk);
    [fkp1, ~] = dynamics_5link(Xkp1, Ukp1);

    Xm = 0.5*(Xk + Xkp1) + (dt/8)*(fk - fkp1);
    Um = 0.5*(Uk + Ukp1);
    [fm, ~] = dynamics_5link(Xm, Um);

    opti.subject_to(-fi(2) <= 0); 
    opti.subject_to(-fi(4) <= 0); 

    opti.subject_to(Xkp1 == Xk + (dt/6)*(fk + 4*fm + fkp1));
    % opti.subject_to(Xkp1 == Xk + dt * fk);
end

% final state constraint
opti.subject_to(X(:,end) == Xf);

% time penalty
obj = obj + case0_k*(tf^2);

% solver setup
opti.minimize(obj);
opts = struct;
% opts.ipopt.hessian_approximation = 'limited-memory';
opts.ipopt.max_iter = 5000;
% opts.ipopt.print_level = 3;
opts.ipopt.acceptable_tol = 1e-3;
opts.ipopt.tol = 1e-4;
opti.solver('ipopt', opts);

% solve
try
    sol = opti.solve();
catch
    sol = opti.debug;
end

Xout = sol.value(X);
Uout = sol.value(U);
tf_val = sol.value(tf);
tout = linspace(0, tf_val, N+1);

save('solved_traj_5link.mat','Xout','Uout','tout','tf_val');

disp('Optimization finished.');
elapsed_time = toc;  
fprintf('Optimization completed in %.2f seconds.\n', elapsed_time);

model = model_biped_5link();
showmotion(model, tout, Xout(1:7, :));