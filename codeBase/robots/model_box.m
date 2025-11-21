function  model = model_box()

persistent last_model;

if length(last_model) ~= 0
  model = last_model;
  return
end

model.NB = 1;
model.parent = 0;

model.jtype = {'R'};		% sacrificial joint replaced by floatbase
model.Xtree = {eye(6)};

% Disc dimensions and inertia

L = 0.2;
W = 0.2;
H = 0.1;

density = 1000;			% kg / m^3
mass = L * W * H * density;
Ixx = mass * (W^2 + H^2) / 12;
Iyy = mass * (L^2 + H^2) / 12;
Izz = mass * (L^2 + W^2) / 12;

model.I = { mcI( mass, [0 0 0], diag([Ixx Iyy Izz]) ) };

% Drawing instructions
model.appearance.base = { 'tiles', [-0.1 2; -0.8 0.8; 0 0], 0.1 };

model.appearance.body{1} = { 'box', 0.5*[L W H; -L -W -H]};

% Camera settings
model.camera.body = 1;
model.camera.direction = [0.2 1 0.1];
model.camera.locus = [0 0.5];

% Ground contact points (have to match how the cylinder is drawn)
X = L/2 * [1 1 1 1 -1 -1 -1 -1];
Y = W/2 * [1 1 -1 -1 1 1 -1 -1 ];
Z = H/2 * [1 -1 -1 1 1 -1 -1 1];

model.gc.point = [X; Y; Z];
model.gc.body = ones(1,length(X));

% Final step: float the base
model = floatbase(model);		% replace joint 1 with a chain of 6
                                        % joints emulating a floating base
last_model = model;
