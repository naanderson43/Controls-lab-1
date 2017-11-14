
K_m = 25; 
K_b = 0.0185;
B = 0.3; 
R_a = 1.25;
L_a = 0; %approximate to zero
J_m = 0.01;
J_l = 6.25;
J_T = J_m + J_l; 

motor_sys = tf(K_m, [L_a*J_T, L_a*B+R_a*J_T, R_a*B+K_m*K_b]); 
gear_sys = tf(1, 20); 
load_sys = tf(1,[1 0]); 
open_loop_tf = motor_sys * gear_sys * load_sys; 

rlocus(open_loop_tf) 

