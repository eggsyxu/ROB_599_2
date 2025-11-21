function  model = model_pendulum()

% parameters
m_link = 1;
len_link = 1;
Iyy_link = 0;

% color
red = [255, 0, 0]/255;
green = [21, 210, 97]/255;
white = [255, 255, 255]/255;
blue = [0, 102, 255]/255;
yellow = [255, 255, 0]/255;

model.mass = m_link;
model.length = len_link;
model.inertia = Iyy_link;

% build robot
model.NB = 1;
nb = 1;

% 1 - link1
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {eye(6)};
model.I(nb) = { mcI( m_link, [0 0 -len_link], diag([0 Iyy_link 0]))};
model.appearance.body{nb} = ...
    { 'colour', blue, ...
      'cyl', [0 0 0; 0 0 -len_link], 0.02, ...
      'sphere', [0 0 -len_link], 0.05};

% Drawing instructions
model.appearance.base = ...
    { 'tiles', [-10 10; 1 1; -10 10], 0.5, ...
      'colour', green, ...
      'cyl', [0 0 0; 0 -0.1 0], 0.05};

% Camera settings
model.camera.body = 0;
model.camera.direction = [0 -0.5 0];
model.camera.locus = [0 0];
model.camera.zoom = 0.4;
