function model = model_3dofArm()

params = params_3dofArm();

model.NB = 3;
nb = 1;
model.linkLength = params.linkLength;

% hip Rz
model.parent(nb) = 0;
model.jtype(nb) = {'Rz'};
model.Xtree(nb) = {plux(eye(3),params.hip_rz_location)};
model.I(nb) = {params.hipRz_inertia};
model.appearance.body{nb} = params.hipRz_appearance;

% hip Ry
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3),params.hip_ry_location)};
model.I(nb) = {params.hipRy_inertia};
model.appearance.body{nb} = params.hipRy_appearance;

% knee Ry
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3),params.knee_ry_location)};
model.I(nb) = {params.kneeRy_inertia};
model.appearance.body{nb} = params.kneeRy_appearance;

% end-effector
model.parent_ee = 3;
model.Xee = {plux(eye(3), [0 0 params.shin_length])};

% base
model.appearance.base = params.base_appearance;

end

function p = params_3dofArm()

% constants
red = [255, 0, 0]/255;
green = [21, 210, 97]/255;
white = [255, 255, 255]/255;
blue = [0, 102, 255]/255;
yellow = [255, 255, 0]/255;
arrowLength = 0.2;
arrowRadius = 0.01;

% motor
motor_mass = 0.2;
motor_thickness = 0.03;
motor_radius = 0.04;
motor_inertia = diag([1e-3 1e-5 1e-5]);

% Rz hip
p.hip_rz_location = [0 0 motor_thickness];
motor_com = [0 0 0];
p.hipRz_inertia = mcI(motor_mass, motor_com, motor_inertia);
p.hipRz_appearance = {'colour', blue, ...
    'cyl', [0 0 0; 0 0 -motor_thickness], motor_radius};

% Ry hip
p.hip_ry_location = [0 0 motor_radius];
motor_com = [0 0 0];
thigh_mass = 0.2;
thigh_length = 0.3;
thigh_rot_inertia = diag([0.05 0.05 1e-4]);
thigh_com = [0 0 0.5*thigh_length];
p.hipRy_inertia = mcI(motor_mass, motor_com, motor_inertia) + ...
                       mcI(thigh_mass, thigh_com, thigh_rot_inertia);
p.hipRy_appearance = {'colour', yellow, ...
    'cyl', [0 0 0; 0 motor_thickness 0], motor_radius, ...
    'cyl', [0 0 0; 0 0 thigh_length], 0.02, ...
    'colour', blue, ...
    'cyl', [0 0 0; 0 -motor_thickness 0], motor_radius};

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
    'colour', red, 'sphere', [0 0 p.shin_length], 0.02, ...
    'colour', red, 'cyl', [0 0 p.shin_length; arrowLength 0 p.shin_length], arrowRadius, ...
    'colour', green, 'cyl', [0 0 p.shin_length; 0 arrowLength p.shin_length], arrowRadius, ...
    'colour', blue, 'cyl', [0 0 p.shin_length; 0 0 arrowLength+p.shin_length], arrowRadius};

% base
p.base_appearance = { 'tiles', [-2 4; -2 2; 0 0], 0.2, ...
    'colour', red, 'cyl', [0 0 0; arrowLength 0 0], arrowRadius, ...
    'colour', green, 'cyl', [0 0 0; 0 arrowLength 0], arrowRadius, ...
    'colour', blue, 'cyl', [0 0 0; 0 0 arrowLength], arrowRadius};

% link length
p.linkLength = [motor_thickness, thigh_length, shin_length];

end