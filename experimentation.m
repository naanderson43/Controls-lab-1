
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
open_loop_sys = motor_sys * gear_sys * load_sys;

rlocus(open_loop_sys) 

Kp= 0.0357; 
Ki= 0; 
Kd= 0;
s=tf('s'); 
controller_sys = Kp + Ki/s + Kd*s; 

open_loop_sys = controller_sys* open_loop_sys;
closed_loop_sys = feedback(open_loop_sys,1); 

rlocus(open_loop_sys)
step(open_loop_sys)
rlocus(closed_loop_sys)
step(closed_loop_sys)



