q0_guess = [0 ; 0.8; 0; pi/3; -2*pi/3; pi/3; -2*pi/3];
prf0 = full(FK_rf(q0_guess));
q0_guess(2) = q0_guess(2) - prf0(2);  
q0 =  q0_guess;

dq0 = zeros(7,1);
X0  = [q0; dq0];

q_takeoff_guess = [0 ; 0.8; -pi/6; pi/6; -pi/3; pi/6; -pi/3]; 
prft = full(FK_rf(q_takeoff_guess));
q_takeoff_guess(1) = q_takeoff_guess(1) - prft(1);
q_takeoff_guess(2) = q_takeoff_guess(2) - prft(2);
q_takeoff = q_takeoff_guess;

g = 9.81;
v_z = 3;
v_x = 2;
t_apex = v_z / g;
x_air = q_takeoff(1) - v_x * t_apex / 2;
z_air = q_takeoff(2) + v_z * t_apex / 2;

q_air_guess = [-0.25; 1.2; 0; pi/6; -pi/3;  pi/6; -pi/3];  
q_air_guess(1) = x_air;
q_air_guess(2) = z_air;
q_air = q_air_guess;

q_land_guess = [0 ; 0.8; pi/6; pi/6; -pi/3; pi/6; -pi/3]
q_land_guess(2) = q_takeoff(2);  
q_land_guess(1) = x_air - v_x * t_apex;
prfl = full(FK_rf(q_land_guess));
q_land_guess(2) = q_land_guess(2) - prfl(2);  
q_land = q_land_guess;

q_final_guess = q0;
prfl = full(FK_rf(q_land));
q_final_guess(1) = q_final_guess(1) + prfl(1);
q_final = q_final_guess;

model = model_biped_5link();
time = [0,     0.3,   0.6,   0.9,   1.2];
Q = [q0, q_takeoff, q_air, q_land, q_final];
showmotion(model, time, Q)  

