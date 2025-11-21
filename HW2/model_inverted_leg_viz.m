function model = model_inverted_leg_viz()

p = get_params();

model.NB = 6;
nb = 1;
model.linkLength = p.linkLength;

% hip Rz
model.parent(nb) = 0;
model.jtype(nb) = {'Rz'};
model.Xtree(nb) = {plux(eye(3),p.hip_rz_location)};
model.I(nb) = {p.hipRz_inertia};
model.appearance.body{nb} = p.hipRz_appearance;

% hip Ry
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3),p.hip_ry_location)};
model.I(nb) = {p.hipRy_inertia};
model.appearance.body{nb} = p.hipRy_appearance;

% knee Ry
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3),p.knee_ry_location)};
model.I(nb) = {p.kneeRy_inertia};
model.appearance.body{nb} = p.kneeRy_appearance;

% desired EE position
nb = nb + 1;
model.parent(nb) = 0;
model.jtype(nb) = {'Px'};
model.Xtree(nb) = {eye(6)};
model.I(nb) = {zeros(6)};

nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Py'};
model.Xtree(nb) = {eye(6)};
model.I(nb) = {zeros(6)};

nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Pz'};
model.Xtree(nb) = {eye(6)};
model.I(nb) = {zeros(6)};
model.appearance.body{nb} = p.EE_viz_appearance;

% end-effector
model.parent_ee = 3;
model.Xee = {plux(eye(3), [0 0 p.shin_length])};

% base
model.appearance.base = p.base_appearance;

end

function p = get_params()

% constants
red = [255, 0, 0]/255;
green = [21, 210, 97]/255;
white = [255, 255, 255]/255;
black = [0, 0, 0];
blue = [0, 102, 255]/255;
yellow = [255, 255, 0]/255;
arrowLength = 0.15;
arrowRadius = 0.01;

% motor
motor_mass = 0.2 * 0;
motor_thickness = 0.03;
motor_radius = 0.04;
motor_inertia = diag([1e-3 1e-5 1e-5]) * 0;

% Rz hip
p.hip_rz_location = [0 0 0];
motor_com = [0 0 0];
offset_link_mass = 0.1;
offset_link_length = 0.08;
offset_link_rot_inertia = diag([0.01, 0.01, 1e-5]);
offset_link_com = [0 0 0.5 * offset_link_length];
p.hipRz_inertia = mcI(motor_mass, motor_com, motor_inertia) + ...
    mcI(offset_link_mass, offset_link_com, offset_link_rot_inertia);
p.hipRz_appearance = {'colour', white, ...
    'cyl', [0 0 -motor_thickness/2; 0 0 motor_thickness/2], motor_radius, ...
    'cyl', [0 0 0; 0 0 offset_link_length], 0.015};

% Ry hip
p.hip_ry_location = [0 0 offset_link_length];
motor_com = [0 0 0];
thigh_mass = 0.2;
thigh_length = 0.3;
thigh_rot_inertia = diag([0.05 0.05 1e-4]);
thigh_com = [0 0 0.5*thigh_length];
p.hipRy_inertia = mcI(motor_mass, motor_com, motor_inertia) + ...
                       mcI(thigh_mass, thigh_com, thigh_rot_inertia);
p.hipRy_appearance = {'colour', yellow, ...
    'cyl', motor_thickness*[0 -0.5 0; 0 0.5 0], motor_radius, ...
    'cyl', [0 0 0; 0 0 thigh_length], 0.015, ...
    % 'colour', blue, ...
    % 'cyl', [0 0 0; 0 -motor_thickness 0], motor_radius
    };

% Ry knee
p.knee_ry_location = [0 0 thigh_length];
motor_com = [0 0 0];
shin_mass = 0.1;
shin_length = 0.25;
p.shin_length = shin_length;
shin_rot_inertia = diag([0.02 0.02 1e-4]);
shin_com = [0 0 0.5*p.shin_length];
p.kneeRy_inertia = mcI(motor_mass, motor_com, motor_inertia) + ...
                      mcI(shin_mass, shin_com, shin_rot_inertia);
p.kneeRy_appearance = {'colour', blue, ...
    'cyl', [0 0 0; 0 0 p.shin_length], 0.015, ...
    'cyl', motor_thickness*[0 -0.5 0; 0 0.5 0], motor_radius, ...
    'colour', green, 'sphere', [0 0 p.shin_length], 0.02, ...
    % 'colour', red, 'cyl', [0 0 p.shin_length; arrowLength 0 p.shin_length], arrowRadius, ...
    % 'colour', green, 'cyl', [0 0 p.shin_length; 0 arrowLength p.shin_length], arrowRadius, ...
    % 'colour', blue, 'cyl', [0 0 p.shin_length; 0 0 arrowLength+p.shin_length], arrowRadius
    };

% EE desired 
p.EE_viz_appearance = {'colour', red, ...
    'sphere', [0, 0, 0], 0.02};

% base
p.base_appearance = { 'tiles', [-2 4; -2 2; 0 0], 0.2, ...
    'colour', red, 'cyl', [0 0 0; arrowLength 0 0], arrowRadius, ...
    'colour', green, 'cyl', [0 0 0; 0 arrowLength 0], arrowRadius, ...
    'colour', blue, 'cyl', [0 0 0; 0 0 arrowLength], arrowRadius
    };

% link length
p.linkLength = [offset_link_length, thigh_length, shin_length];

end