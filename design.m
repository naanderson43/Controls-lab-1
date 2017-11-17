clf
clear('all');


natural_frequency = 10; 
damping_ratio = 0.707; 

syms s k1 k2 

% TO ERASE 
% motor_sys = 465/(s + 84.8);
% gear_sys = 1/20; 
% load_sys = 1/s; 
% open_loop_sys = (k1 * motor_sys * gear_sys * load_sys)/(1 + motor_sys*k2 + k1 * motor_sys * gear_sys * load_sys);
% open_loop_sys_damp = natural_frequency^2/(s^2 + 2*damping_ratio*natural_frequency*s+(natural_frequency^2));


equation1 = 23.25*k1 == natural_frequency^2; 
equation2 = 84.8 + 465*k2 == 2 * damping_ratio * natural_frequency; 
% [y1, y2] = solve(open_loop_sys == open_loop_sys_damp, [k1 k2]);
[k1, k2] = solve([equation1 equation2], [k1 k2]);
% calculator values matlab has a roundoff 
% k1 = 4.30108
% k2 = -0.151957

motor_sys = tf(465 , [1, 84.8]);
gear_sys = tf(1,20);
load_sys = tf(1, [1 0]);
k1 = tf(double(k1)); 
k2 = tf(double(k2)); 

open_loop_sys = (k1 * motor_sys * gear_sys * load_sys)/(1 + motor_sys*k2 + k1 * motor_sys * gear_sys * load_sys);

figure(1); 
step(open_loop_sys); 



