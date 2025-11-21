function animate_quadrotor(t, X, U, XDes, p)

if nargin < 6
    saveVideo = false;
end

if nargin < 5
    error('Missing input argument p (parameter struct).');
end

if ~isstruct(p)
    error('Input p must be a struct (e.g., p.halfWidth = 0.2).');
end

halfWidth = p.halfWidth;

figure('Color','w','Position',[100 100 700 500]);
axis equal; grid on; hold on;
xlabel('x (m)');
ylabel('z (m)');
title('Quadrotor Animation');
axis([-1.5 1.5 0 2]);


hw3Path = fullfile('E:\OneDrive\Desktop\study\Course\ROB_599_2\HW3');
if ~exist(hw3Path, 'dir')
    mkdir(hw3Path);
end
outputFile = fullfile(hw3Path, 'quadrotor_sim.mp4');
v = VideoWriter(outputFile, 'MPEG-4');
v.FrameRate = 1 / mean(diff(t));


for k = 1:5:length(t)
    cla;

    % 
    x = X(k,1);
    z = X(k,2);
    th = X(k,3);

    % 
    R = [cos(th) -sin(th); sin(th) cos(th)];
    arm = R * [-halfWidth, halfWidth; 0, 0];

    % 
    plot(x + arm(1,:), z + arm(2,:), 'k-', 'LineWidth', 3);
    plot(x, z, 'ro', 'MarkerFaceColor','r', 'MarkerSize', 6);

    % 
    fscale = 0.0005;
    u1 = U(min(k,size(U,1)),1);
    u2 = U(min(k,size(U,1)),2);
    f1 = fscale * u1 * R * [0;1];
    f2 = fscale * u2 * R * [0;1];
    plot(x + arm(1,1) + [0 f1(1)], z + arm(2,1) + [0 f1(2)], 'b', 'LineWidth', 2);
    plot(x + arm(1,2) + [0 f2(1)], z + arm(2,2) + [0 f2(2)], 'b', 'LineWidth', 2);

    % XDes
    if ~isempty(XDes)
        plot(XDes(1), XDes(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    end

    % 
    text(-1.3, 1.8, sprintf('t = %.2f s', t(k)), 'FontSize', 10);

    drawnow;

    if saveVideo
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
end

if saveVideo
    close(v);
    disp(' quadrotor_sim.mp4');
end
end
