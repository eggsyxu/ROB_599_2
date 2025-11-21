function  model = model_biped_5link()

% color
red = [255, 0, 0]/255;
green = [21, 210, 97]/255;
white = [255, 255, 255]/255;
blue = [0, 102, 255]/255;
yellow = [255, 255, 0]/255;
radius_cyl = 0.02;

% parameters
m_body = 10;model.m_body = m_body;
m_thigh = 1;
m_shin = 0.5;
len_body = 0.4;
len_thigh = 0.4;
len_shin = 0.4;
com_body = len_body * 0.5;
com_thigh = len_thigh * 0.5;
com_shin = len_shin * 0.5;
Iyy_body = 0.05;model.Iyy_body = Iyy_body;
Iyy_thigh = 0.01;
Iyy_shin = 0.005;

model.mass = m_body + 2 * (m_thigh + m_shin);
model.NB = 7;
model.mu = 0.5;

gc_body = [];
gc_point = [];

%% Build the model
nb = 1;

% 1 - Px
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Px'};
model.Xtree(nb) = {eye(6)};
model.com(nb) = {[0; 0; 0]};
model.I(nb) = { zeros(6) };

% 2 - Pz
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Pz'};
model.Xtree(nb) = {eye(6)};
model.com(nb) = {[0; 0; 0]};
model.m_link(nb) = 0;
model.I(nb) = { zeros(6) };

% 3 - torso
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3), [0; 0; 0])};
model.com(nb) = {[0; 0; com_body]};
model.m_link(nb) = m_body;
model.I(nb) = { mcI( m_body, [0 0 com_body], diag([0 Iyy_body 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', red, 'cyl', [0 0 0; 0 0 len_body], radius_cyl};

gc_body = [gc_body, nb];
gc_point = [gc_point, [0; 0; 0]];

% 4 - hip
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3), [0; 0; 0])};
model.com(nb) = {[0; 0; -com_thigh]};
model.m_link(nb) = m_thigh;
model.I(nb) = { mcI( m_thigh, [0 0 -com_thigh], diag([0 Iyy_thigh 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', green, 'cyl', [0 0 0; 0 0 -len_thigh], radius_cyl};

gc_body = [gc_body, nb];
gc_point = [gc_point, [0; 0; -len_thigh]];

% 5 - knee
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3), [0; 0; -len_thigh])};
model.com(nb) = {[0; 0; -com_shin]};
model.m_link(nb) = m_shin;
model.I(nb) = { mcI( m_shin, [0 0 -com_shin], diag([0 Iyy_shin 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', green, 'cyl', [0 0 0; 0 0 -len_shin], radius_cyl};

gc_body = [gc_body, nb];
gc_point = [gc_point, [0; 0; -len_shin]];

% 6 - hip
nb = nb + 1;
model.parent(nb) = 3;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3), [0; 0; 0])};
model.com(nb) = {[0; 0; -com_thigh]};
model.m_link(nb) = m_thigh;
model.I(nb) = { mcI( m_thigh, [0 0 -com_thigh], diag([0 Iyy_thigh 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', blue, 'cyl', [0 0 0; 0 0 -len_thigh], radius_cyl};

gc_body = [gc_body, nb];
gc_point = [gc_point, [0; 0; -len_thigh]];

% 7 - knee
nb = nb + 1;
model.parent(nb) = nb - 1;
model.jtype(nb) = {'Ry'};
model.Xtree(nb) = {plux(eye(3), [0; 0; -len_thigh])};
model.com(nb) = {[0; 0; -com_shin]};
model.m_link(nb) = m_shin;
model.I(nb) = { mcI( m_shin, [0 0 -com_shin], diag([0 Iyy_shin 0]) ) };
model.appearance.body{nb} = ...
    { 'colour', blue, 'cyl', [0 0 0; 0 0 -len_shin], radius_cyl};

gc_body = [gc_body, nb];
gc_point = [gc_point, [0; 0; -len_shin]];

%%
% Drawing instructions
model.appearance.base = ...
    { 'tiles', [-1 1; -0.8 0.8; 0 0], 0.1};

% Camera settings
model.camera.body = 0;
model.camera.direction = [0 -0.5 0.2];
model.camera.locus = [0 -0.5];
model.camera.zoom = 0.3;

% Contact points
model.gc.body = gc_body;
model.gc.point = gc_point;


end
