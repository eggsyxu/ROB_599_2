function generate_kinematics_and_dynamics()
syms q [5,1] real
syms dq [5,1] real

% syms g [1,1] real
% syms l [3,1] real
% syms M [3,1] real
% syms I [3,1] real

% params = [g;l;M;I];

g = 9.81;
m_body = 10;
m_thigh = 1;
m_shin = 0.5;
M = [m_body, m_thigh, m_shin];
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
I = [Iyy_body, Iyy_thigh, Iyy_shin];

N = 5;

rot = @(x)[cos(x) sin(x); -sin(x) cos(x)];

T01 = [rot(q3) [q1; q2]; 0 0 1];
T12 = [rot(q4) [0;0]; 0 0 1];
T23 = [rot(q5) [0;-l(2)]; 0 0 1];
T34 = [eye(2) [0;-l(3)]; 0 0 1];

T04 = T01 * T12 * T23 * T34;
pf = T04(1:2, 3);

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;

p1 = T01(1:2,3);
p2 = T02(1:2,3);
p3 = T03(1:2,3);
p4 = T04(1:2,3);

pcom_torso = p1 + 0.5 * rot(q3) * [0; len_body];
pcom = [pcom_torso, 0.5*(p2+p3), 0.5*(p3+p4)];
pcom = simplify(pcom);
vcom = reshape(jacobian(pcom(:),q) * dq, size(pcom));

Jf = jacobian(pf, q);
dJf_dq = sym(zeros(2, 5, 5));
for ii = 1:5
    dJf_dq(:, :, ii) = diff(Jf, q(ii));
end

dJf_dt = sym(zeros(size(Jf)));
for ii = 1:5
    dJf_dt = dJf_dt + squeeze(dJf_dq(:, :, ii)) * dq(ii);
end

matlabFunction(pf, "File","FK_foot.m", "Vars",{q});
matlabFunction(Jf, "File","dFK_dq_foot.m", "Vars",{q});
matlabFunction(dJf_dt, "File","Jc_dot_func.m", "Vars",{q, dq});

%%
M_ = M';
I_ = I';
PE = pcom(2, :) * M_ * g;

w_ = [dq3 dq3+dq4 dq3+dq4+dq5]';

KE = sym(0);
for ii = 1:3
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

matlabFunction(H, 'File', 'H_3link', 'Vars',{q});
matlabFunction(bias, 'File', 'bias_3link', 'Vars',{q,dq});

end