function robot = params_biped()
    % color
    red = [255, 0, 0]/255;
    green = [21, 210, 97]/255;
    white = [255, 255, 255]/255;
    blue = [0, 102, 255]/255;
    yellow = [255, 255, 0]/255;

    % robot parameters
    hip_width = 0.15;
    tail_length = 0.5;

    % torso
    torso_mass = 5;
    torso_length = 0.2;
    torso_width = 0.20;
    torso_height = 0.1;
    torso_rot_inertia = diag([0.1 0.1 0.1]);
    torso_com = [0 0 0];
    robot.torso_inertia = mcI(torso_mass, torso_com, torso_rot_inertia);
    
    pDiag1 = 0.5 * [torso_length torso_width torso_height];
    pDiag2 = -0.5 * [torso_length torso_width torso_height];
    robot.torso_appearance = ...
        {'colour', green,...
         'box', [pDiag1; pDiag2]};

    % hip Rx right
    robot.right_hip_rx_location = [0 -0.5*hip_width -0.5*torso_height];
    hipRx_mass = 0.1;
    hipRx_length = 0.03;
    hipRx_rot_inertia = diag([1e-3 1e-5 1e-5]);
    hipRx_com = [0 0 0];
    robot.hipRx_inertia = mcI(hipRx_mass, hipRx_com, hipRx_rot_inertia);
    robot.hipRx_appearance = ...
        {'colour', blue,...
         'cyl', [hipRx_length 0 0; 0 0 0], 0.04};

    % hip Ry right
    robot.hip_ry_location = [0 0 0];
    thigh_mass = 0.1;
    thigh_length = 0.23;
    thigh_rot_inertia = diag([0.05 0.05 1e-4]);
    thigh_com = [0 0 -0.5*thigh_length];
    robot.thigh_inertia = mcI(thigh_mass, thigh_com, thigh_rot_inertia);
    robot.thigh_appearance = ...
        {'colour', blue,...
         'cyl', [0 0 0; 0 0 -thigh_length], 0.02};

    % knee Ry right
    robot.knee_ry_location = [0 0 -thigh_length];
    shin_mass = 0.1;
    shin_length = 0.23;
    shin_rot_inertia = diag([0.05 0.05 1e-4]);
    shin_com = [0 0 -0.5*shin_length];
    robot.shin_inertia = mcI(shin_mass, shin_com, shin_rot_inertia);
    robot.shin_appearance = ...
        {'colour', yellow,...
         'cyl', [0 0 0; 0 0 -shin_length], 0.015};

    % tail
    robot.tail_ry_location = [-0.5*torso_length 0 0.5*torso_height];
    tail_mass = 0.5;
    tail_rot_inertia = diag([1e-4 5e-3 5e-3]);
    tail_com = [-0.8*tail_length 0 0];
    robot.tail_inertia = mcI(tail_mass, tail_com, tail_rot_inertia);
    robot.tail_appearance = ...
        {'colour', blue,...
         'cyl', [0 0.01 0; 0 -0.01 0], 0.03,...
         'cyl', [0 0 0; -tail_length 0 0], 0.01,...
         'cyl', [-tail_length 0 0; -tail_length+0.05 0 0], 0.02};

    %% contact points
    % torso
    X = torso_length/2 * [1 1 1 1 -1 -1 -1 -1];
    Y = torso_width/2 * [1 1 -1 -1 1 1 -1 -1 ];
    Z = torso_height/2 * [1 -1 -1 1 1 -1 -1 1];
    robot.gc_body = ones(1, length(X));
    robot.gc_points = [X; Y; Z];

    % right leg
    robot.gc_body = [robot.gc_body, 3];
    robot.gc_points = [robot.gc_points, [0; 0; -thigh_length]];
    
    robot.gc_body = [robot.gc_body, 4];
    robot.gc_points = [robot.gc_points, [0; 0; -shin_length]];

    % left leg
    robot.gc_body = [robot.gc_body, 6];
    robot.gc_points = [robot.gc_points, [0; 0; -thigh_length]];
    
    robot.gc_body = [robot.gc_body, 7];
    robot.gc_points = [robot.gc_points, [0; 0; -shin_length]];

    % tail
    robot.gc_body = [robot.gc_body, 8];
    robot.gc_points = [robot.gc_points, [-tail_length; 0; 0]];

end