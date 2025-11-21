function [Xout, Uout, tout, tf_val] = generate_traj_5link(q0, qf, N, cost_idx, case0_k)
% =========================================================================
% Input:
%   q0        : 7x1 initial configuration [x, y, theta, q1_r, q2_r, q1_l, q2_l]
%   qf        : 7x1 final configuration
%   N         : number of collocation points, default = 50
%   cost_idx  : cost function selector
%                 0 -> Yutong cost function 
%                 1 -> paper cost function
%   case0_k   : cost scaling for case 0, default = 0.1
%
% Output:
%   Xout, Uout, tout, tf_val
% =========================================================================

    % ======= Setup Path and CasADi =======
    run('../../setup.m');
    import casadi.*
    addpath(genpath(pwd));

    % ======= Parameters =======
    ds = 1/N;
    dq0 = zeros(7,1);
    dqf = zeros(7,1);
    X0 = [q0; dq0];
    Xf = [qf; dqf];
    final_time = 5;

    % weight matrices for case 1 (LQR-type)
    xmax = [0.02, 0.02, deg2rad(3), deg2rad(5), deg2rad(5), deg2rad(5), deg2rad(5), ...
            0.2,  0.2,  deg2rad(20), 0.5, 0.5, 0.5, 0.5];
    Q = diag(1./(xmax.^2));
    umax = [120 120 120 120];
    R = diag(1./(umax.^2)) * 5;

    % ======= Define Opti Variables =======
    opti = casadi.Opti();
    X  = opti.variable(14, N+1);   % [q; dq]
    U  = opti.variable(4,  N+1);   % 4 joint torques
    tf = opti.variable(1,1);
    dt = tf * ds;
    obj = MX(0);

    % ======= Boundary Conditions =======
    opti.subject_to(X(1:7,1) == q0);
    dq0_var = X(8:14,1);
    v0 = dFK_rf_dq(X(1:7,1)) * dq0_var;
    opti.subject_to(v0 == [0; 0]);      % stick contact
    opti.subject_to(tf >= 1);
    opti.subject_to(tf <= final_time);

    % ======= Dynamics and Cost Loop =======
    for k = 1:N
        Xk   = X(:,k);
        Uk   = U(:,k);
        Xkp1 = X(:,k+1);
        Ukp1 = U(:,k+1);

        % ===== Cost Function Switch =====
        switch cost_idx
            case 0
                obj = obj + ...
                    case0_k*(Uk'*Uk) + ...
                    case0_k*((Ukp1-Uk)'*(Ukp1-Uk)) + ...
                    case0_k*(FK_rf(Xk(1:7))'*FK_rf(Xk(1:7)));
            case 1
                obj = obj + Xk' * Q * Xk + Uk' * R * Uk;
            otherwise
                error('Invalid cost_idx: must be 0 or 1.');
        end

        % ===== Dynamics Integration (Hermite-Simpson) =====
        [fk,   ~] = dynamics_5link(Xk,   Uk);
        [fkp1, ~] = dynamics_5link(Xkp1, Ukp1);
        Xm = 0.5*(Xk + Xkp1) + (dt/8)*(fk - fkp1);
        Um = 0.5*(Uk + Ukp1);
        [fm, ~] = dynamics_5link(Xm, Um);
        opti.subject_to(Xkp1 == Xk + (dt/6)*(fk + 4*fm + fkp1));
    end

    % ===== Add Time Penalty (once, outside loop) =====
    obj = obj + 0.01*(tf^2);

    % ===== Final State Constraint =====
    opti.subject_to(X(:,end) == Xf);

    % ======= Solver Setup =======
    opti.minimize(obj);
    opts = struct;
    opts.ipopt.max_iter = 3000;
    opts.ipopt.print_level = 3;
    opts.ipopt.tol = 1e-6;
    opts.ipopt.linear_solver = 'mumps';
    opti.solver('ipopt', opts);

    % ======= Initial Guess =======
    opti.set_initial(tf, 2);
    opti.set_initial(X, repmat(X0,1,N+1));
    opti.set_initial(U, zeros(4,N+1));

    % ======= Solve =======
    sol = opti.solve();
    Xout = sol.value(X);
    Uout = sol.value(U);
    tf_val = sol.value(tf);
    tout = linspace(0, tf_val, N+1);

    % ======= Save Results =======
    timestamp = datestr(now,'yyyymmdd_HHMMSS');
    filename = sprintf('traj5link_case%d_%s.mat', cost_idx, timestamp);
    save(filename, 'Xout','Uout','tout','tf_val','q0','qf','cost_idx','case0_k','N');
    fprintf('[DONE] Saved: %s\n', filename);
    fprintf('Final time: %.3f s | Max torque: %.2f Nm\n', tf_val, max(abs(Uout(:))));
end
