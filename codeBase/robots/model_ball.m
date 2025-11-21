function  model = model_ball()

persistent last_model;

if length(last_model) ~= 0
  model = last_model;
  return
end

model.NB = 1;
model.parent = 0;

model.jtype = {'R'};		% sacrificial joint replaced by floatbase
model.Xtree = {eye(6)};

radius = 0.1;

density = 1000;			% kg / m^3
mass = 4/3 * pi * radius^3 * density;
Ixx = 2/5 * mass * radius^2;

model.I = { mcI( mass, [0 0 0], diag([Ixx Ixx Ixx]) ) };

% Drawing instructions
model.appearance.base = { 'tiles', [-0.1 2; -0.8 0.8; 0 0], 0.1 };

model.appearance.body{1} = { 'sphere', zeros(3,1), radius};

% Camera settings
model.camera.body = 1;
model.camera.direction = [0.2 1 0.1];
model.camera.locus = [0 0.5];
model.camera.zoom = 0.2;

model.gc.point = [0; 0; 0];
model.gc.body = 1;

% Final step: float the base
model = floatbase(model);		% replace joint 1 with a chain of 6
                                        % joints emulating a floating base
last_model = model;
