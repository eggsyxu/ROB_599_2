function  model = model_tailedOneLeg_vertical()

% parameters
m_body = 2;
m_thigh = 0.5;
m_shin = 0.3;
m_tail = 0.5;
dim_body = [0.2, 0.1, 0.1];
len_thigh = 0.3;
len_shin = 0.3;
len_tail = 0.5;
com_thigh = 0.3;
com_shin = 0.2;
com_tail = 0.8;
Iyy_thigh = 0.01;
Iyy_shin = 0.005;
Iyy_tail = 0.01;

M = [m_body, m_thigh, m_shin, m_tail];
l = [len_thigh, len_shin, len_tail];
r = [com_thigh, com_shin, com_tail];
I = [Iyy_thigh, Iyy_shin, Iyy_tail];

model.params = [M, l, r, I]';
model.Mc = diag([1 -1 -1 -1]);

mass_2RB = [m_body+m_thigh+m_shin, m_tail];
model.params_2RB = [mass_2RB, len_tail, com_tail, Iyy_tail]';

% color
red = [255, 0, 0]/255;
green = [21, 210, 97]/255;
white = [255, 255, 255]/255;
blue = [0, 102, 255]/255;
yellow = [255, 255, 0]/255;

% build robot
model.NB = 4;
nb = 1;
gc_body = [];
gc_point = [];

% 1 - body
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Pz'};
model.Xtree(nb) = {eye(6)};
model.I(nb) = { mcI( m_body, [0 0 0], diag([0 0 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', blue, 'box', 0.5*[-dim_body; dim_body]};

gc_body = [gc_body, nb];
gc_point = [gc_point, [0; 0; 0]];

% 2 - hip
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3), [0; 0; 0])};
model.I(nb) = { mcI( m_thigh, [0 0 -com_thigh*len_thigh], diag([0 Iyy_thigh 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', green, 'cyl', [0 0 0; 0 0 -len_thigh], 0.02};

gc_body = [gc_body, nb];
gc_point = [gc_point, [0; 0; -len_thigh]];

% 3 - knee
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3), [0; 0; -len_thigh])};
model.I(nb) = { mcI( m_shin, [0 0 -com_shin*len_shin], diag([0 Iyy_shin 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', green, 'cyl', [0 0 0; 0 0 -len_shin], 0.02};

gc_body = [gc_body, nb];
gc_point = [gc_point, [0; 0; -len_shin]];

% 4 - tail
nb = nb + 1;
model.parent(nb) = 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3), [0; 0; 0])};
model.I(nb) = { mcI( m_tail, [-com_tail*len_tail 0 0], diag([0 Iyy_tail 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', green, 'cyl', [0 0 0; -len_tail 0 0], 0.02, ...
      'colour', blue, 'cyl', [-len_tail 0 0; -len_tail+0.06 0 0], 0.04};

gc_body = [gc_body, nb];
gc_point = [gc_point, [-len_tail; 0; 0]];

% Drawing instructions
model.appearance.base = ...
    { 'tiles', [-1 1; -0.8 0.8; 0 0], 0.1, ...
      'colour', red, ...
      'cyl', [0 0 0; 0 0 2], 0.02};

% Camera settings
model.camera.body = 0;
model.camera.direction = [0 -0.5 0.2];
model.camera.locus = [0 -0.2];
model.camera.zoom = 0.4;

% Contact points
model.gc.body = gc_body;
model.gc.point = gc_point;

