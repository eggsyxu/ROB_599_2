function  model = model_cartpole()

persistent last_model;

if length(last_model) ~= 0
  model = last_model;
  return
end

model.name = 'cartpole';

% parameters
m_cart = 1;
m_pole = 0.5;
dim_cart = [0.2 0.1 0.1];
len_pole = 0.7;
Iyy_pole = 0.01;

model.params = [m_cart, m_pole, len_pole, 0.5, Iyy_pole]';

% color
red = [255, 0, 0]/255;
green = [21, 210, 97]/255;
white = [255, 255, 255]/255;
blue = [0, 102, 255]/255;
yellow = [255, 255, 0]/255;

% build robot
model.NB = 2;
model.Nu = 1;   % number of actuated joints
model.B = [1;0];
nb = 1;

% 1 - cart
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Px'};
model.Xtree(nb) = {eye(6)};
model.I(nb) = { mcI( m_cart, [0 0 0], diag([0 0 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', blue, 'box', [-0.5*dim_cart; 0.5*dim_cart]};

% 2 - pole
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3), [0; 0; 0])};
model.I(nb) = { mcI( m_pole, [0 0 -0.5*len_pole], diag([0 Iyy_pole 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', green, 'cyl', [0 0 0; 0 0 -len_pole], 0.02};

% Drawing instructions
model.appearance.base = ...
    { 'tiles', [-1 1; -0.8 0.8; -1 -1], 0.1, ...
      'colour', red, ...
      'cyl', [-2 0 0; 2 0 0], 0.02};

% Camera settings
model.camera.body = 0;
model.camera.direction = [0 -0.5 0.2];
model.camera.locus = [0 0];
model.camera.zoom = 0.3;

% model.gc.point = [0; 0; 0];
% model.gc.body = 1;

last_model = model;
