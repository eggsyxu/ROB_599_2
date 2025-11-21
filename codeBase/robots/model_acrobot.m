function  model = model_acrobot()

% parameters
m_link1 = 0.5;
m_link2 = 1;
len_link1 = 0.5;
len_link2 = 1;
Iyy_link1 = 1e-3;
Iyy_link2 = 2e-3;

% color
red = [255, 0, 0]/255;
green = [21, 210, 97]/255;
white = [255, 255, 255]/255;
blue = [0, 102, 255]/255;
yellow = [255, 255, 0]/255;

% build robot
model.name = 'acrobot';
model.B = [0; 1];
model.NB = 2;
nb = 1;
gc_body = [];
gc_point = [];

% 1 - link1
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {eye(6)};
model.I(nb) = { mcI( m_link1, [0 0 -0.5*len_link1], diag([0 Iyy_link1 0]))};
model.appearance.body{nb} = ...
    { 'colour', blue, 'cyl', [0 0 0; 0 0 -len_link1], 0.02};

% 2 - link2
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3), [0; 0; -len_link1])};
model.I(nb) = { mcI( m_link2, [0 0 -0.5*len_link2], diag([0 Iyy_link2 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', red, 'cyl', [0 0 0; 0 0 -len_link2], 0.02};

gc_body = [gc_body, nb];
gc_point = [gc_point, [0; 0; -len_link2]];

% Drawing instructions
model.appearance.base = ...
    { 'tiles', [-1 1; -0.8 0.8; -1.5 -1.5], 0.1, ...
      'colour', green, ...
      'cyl', [0 0 0; 0 -0.1 0], 0.05};

% Camera settings
model.camera.body = 0;
model.camera.direction = [0 -0.5 0.2];
model.camera.locus = [0 0];
model.camera.zoom = 0.5;

% Contact points
model.gc.body = gc_body;
model.gc.point = gc_point;

