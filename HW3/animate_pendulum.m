function [Xout, tauout] = animate_pendulum(X0, opt_sol, Nstep, p)
% animate_pendulum(X0, opt_sol, Nstep, p)
%
% X0: 初始状态 [q; dq; s]
% opt_sol: [co_tau, tf]
% Nstep: 步数 (p.Nseg * p.Nms)
% p: 参数结构体 (含 L, nco, mass, g)
%
% 返回:
% Xout: 状态轨迹
% tauout: 扭矩轨迹
%
% 动画保存在 HW3 目录下

    % 提取参数
    L = p.L;
    nco = p.nco;
    tf = opt_sol(nco+1);
    co_tau = opt_sol(1:nco);

    % 时间与初始化
    t = linspace(0, tf, Nstep);
    ds = 1/Nstep;
    X = X0;
    Xout = X0';
    tauout = [];

    % 创建动画窗口
    figure('Color','w','Position',[200 200 700 500]);
    axis equal; grid on; hold on;
    xlabel('x (m)');
    ylabel('y (m)');
    title('Pendulum Animation');
    axis([-L-0.1 L+0.1 -L-0.1 L+0.1]);

    % 准备视频保存路径
    hw3Path = fullfile('E:\OneDrive\Desktop\study\Course\ROB_599_2\HW3');
    if ~exist(hw3Path, 'dir')
        mkdir(hw3Path);
    end
    outputFile = fullfile(hw3Path, 'pendulum_sim.mp4');
    v = VideoWriter(outputFile, 'MPEG-4');
    v.FrameRate = 30;
    open(v);

    % 主循环
    for k = 1:Nstep
        s = (k-1) * ds;
        [dXdt, tau] = dynamics(s, X, opt_sol, p);
        X = X + dXdt * ds;  % 简单Euler步
        Xout = [Xout; X'];
        tauout = [tauout; tau];

        % 绘图
        cla;
        q = X(1);
        % Pendulum end
        origin = [0; 0];
        pend_tip = [L * sin(q); -L * cos(q)];
        plot([origin(1) pend_tip(1)], [origin(2) pend_tip(2)], 'k-', 'LineWidth', 3);
        plot(pend_tip(1), pend_tip(2), 'ro', 'MarkerFaceColor','r', 'MarkerSize', 8);
        text(-0.4, 0.4, sprintf('t = %.2f s', t(k)), 'FontSize', 10);

        drawnow;
        frame = getframe(gcf);
        writeVideo(v, frame);
    end

    close(v);
    disp(['✅ 视频已保存到：' outputFile]);
end
