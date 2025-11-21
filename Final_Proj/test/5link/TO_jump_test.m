% Path
run('E:\OneDrive\Desktop\study\Course\ROB_599_2\codeBase\setup.m');  % Replace your own path
import casadi.*
addpath(genpath(pwd));

% time step parameters
N = 100; 
N_d = 40;
N_f = 30;
N_l = 30;
ds = 1/N;  

k1 = N_d;
k2 = N_d + N_f;
k3 = N;

% states setup
q0_guess = [0 ; 0.8; 0; pi/3; -2*pi/3; pi/3; -2*pi/3];
prf0 = full(FK_rf(q0_guess));
q0_guess(2) = q0_guess(2) - prf0(2);  
q0 =  q0_guess;

dq0 = zeros(7,1);
X0  = [q0; dq0];

q_takeoff_guess = [0 ; 0.8; -pi/6; pi/6; -pi/3; pi/6; -pi/3]; 
prft = full(FK_rf(q_takeoff_guess));
q_takeoff_guess(1) = q_takeoff_guess(1) - prft(1);
q_takeoff_guess(2) = q_takeoff_guess(2) - prft(2);
q_takeoff = q_takeoff_guess;
dq_takeoff = [-4; 3; 0; 0; 0; 0; 0];
X_takeoff  = [q_takeoff; dq_takeoff];

g = 9.81;
v_z = 3;
v_x = 4;
t_apex = v_z / g;
x_air = q_takeoff(1) - v_x * t_apex / 2;
% z_air = q_takeoff(2) + v_z * t_apex / 2;
% 
% q_air_guess = [-0.25; 1.2; 0; pi/6; -pi/3;  pi/6; -pi/3];  
% q_air_guess(1) = x_air;
% q_air_guess(2) = z_air;
% q_air = q_air_guess;

q_land_guess = q_air;  
q_land_guess(1) = x_air - v_x * t_apex / 2;
prfl = full(FK_rf(q_land_guess));
q_land_guess(2) = q_land_guess(2) - prfl(2);  
q_land_pre = q_land_guess;
dq_land_pre = [-4; 3; 0; 0; 0; 0; 0];
X_land = [q_land; dq_land];

q_final_guess = q0;
q_final_guess(1) = q_land(1); 
q_final = q_final_guess;

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