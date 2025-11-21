function generate_kinematics_and_dynamics()
syms q [7,1] real
syms dq [7,1] real

% syms g [1,1] real
% syms l [3,1] real
% syms M [3,1] real
% syms I [3,1] real

% params = [g;l;M;I];

g = 9.81;
m_body = 10;
m_thigh = 1;
m_shin = 0.5;
M = [m_body, m_thigh, m_shin, m_thigh, m_shin];
len_body = 0.4;
len_thigh = 0.4;
len_shin = 0.4;
l = [len_body, len_thigh, len_shin];
% com_body = len_body * 0.5;
% com_thigh = len_thigh * 0.5;
% com_shin = len_shin * 0.5;
Iyy_body = 0.05;
Iyy_thigh = 0.01;
Iyy_shin = 0.005;
I = [Iyy_body, Iyy_thigh, Iyy_shin, Iyy_thigh, Iyy_shin];

N = 7;

rot = @(x)[cos(x) sin(x); -sin(x) cos(x)];

T01 = [rot(q3) [q1; q2]; 0 0 1];
T12 = [rot(q4) [0;0]; 0 0 1];
T23 = [rot(q5) [0;-l(2)]; 0 0 1];
T34 = [eye(2) [0;-l(3)]; 0 0 1];
T15 = [rot(q6) [0;0]; 0 0 1];
T56 = [rot(q7) [0;-l(2)]; 0 0 1];
T67 = [eye(2) [0;-l(3)]; 0 0 1];

T04 = T01 * T12 * T23 * T34;
T07 = T01 * T15 * T56 * T67;
p_rf = T04(1:2, 3);
p_lf = T07(1:2, 3);


T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T01 * T15;
T06 = T05 * T56;
T07 = T06 * T67;

p1 = T01(1:2,3);
p2 = T02(1:2,3);
p3 = T03(1:2,3);
p4 = T04(1:2,3);
p5 = T05(1:2,3);
p6 = T06(1:2,3);
p7 = T07(1:2,3);

pcom_torso = p1 + 0.5 * rot(q3) * [0; len_body];
pcom = [pcom_torso, 0.5*(p2+p3), 0.5*(p3+p4), 0.5*(p5+p6), 0.5*(p6+p7)];
pcom = simplify(pcom);
vcom = reshape(jacobian(pcom(:),q) * dq, size(pcom));

J_rf = jacobian(p_rf, q);
dJ_rf_dq = sym(zeros(2, 7, 7));
for ii = 1:5
    dJ_rf_dq(:, :, ii) = diff(J_rf, q(ii));
end

dJ_rf_dt = sym(zeros(size(J_rf)));
for ii = 1:5
    dJ_rf_dt = dJ_rf_dt + squeeze(dJ_rf_dq(:, :, ii)) * dq(ii);
end

J_lf = jacobian(p_lf, q);
dJ_lf_dq = sym(zeros(2, 7, 7));
for ii = 1:5
    dJ_lf_dq(:, :, ii) = diff(J_lf, q(ii));
end

dJ_lf_dt = sym(zeros(size(J_lf)));
for ii = 1:5
    dJ_lf_dt = dJ_lf_dt + squeeze(dJ_lf_dq(:, :, ii)) * dq(ii);
end

matlabFunction(p_rf, "File","FK_rf.m", "Vars",{q});
matlabFunction(J_rf, "File","dFK_rf_dq.m", "Vars",{q});
matlabFunction(dJ_rf_dt, "File","Jc_dot_rf_func.m", "Vars",{q, dq});

matlabFunction(p_lf, "File","FK_lf.m", "Vars",{q});
matlabFunction(J_lf, "File","dFK_lf_dq.m", "Vars",{q});
matlabFunction(dJ_lf_dt, "File","Jc_dot_lf_func.m", "Vars",{q, dq});

%%
M_ = M';
I_ = I';
PE = pcom(2, :) * M_ * g;

w_ = [dq3 dq3+dq4 dq3+dq4+dq5 dq3+dq6 dq3+dq6+dq7]';

KE = sym(0);
for ii = 1:5
    KE = KE + 0.5 * vcom(:,ii)' * M_(ii) * vcom(:,ii);
    KE = KE + 0.5 * w_(ii)' * I_(ii) * w_(ii);
end

G = jacobian(PE, q).';
H = simplify(hessian(KE,dq));

syms C [N, N] real
for k = 1:N
    for j = 1:N
        C(k,j) = 0;
        for i = 1:N
            xkji = jacobian(H(k,j),q(i));
            xkij = jacobian(H(k,i),q(j));
            xijk = jacobian(H(i,j),q(k));
            C(k,j) = C(k,j) + 1/2 * (xkji+xkij-xijk) * dq(i);
        end
    end
end

bias = C * dq + G;

matlabFunction(H, 'File', 'H_5link', 'Vars',{q});
matlabFunction(bias, 'File', 'bias_5link', 'Vars',{q,dq});

end