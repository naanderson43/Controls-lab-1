clf
clear('all');

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
closed_loop_sys = feedback(open_loop_sys,1); 

figure(1); 
rlocus(open_loop_sys); 
figure(2); 
step(closed_loop_sys); 

Kp= 0.0357; %obtained from the rootlocus 
Ki= 0; 
Kd= 0;
s=tf('s'); 
controller_sys = Kp + Ki/s + Kd*s; 

open_loop_kp_sys = controller_sys* open_loop_sys;
closed_loop_kp_sys = feedback(open_loop_kp_sys,1); 

figure(3); 
rlocus(open_loop_kp_sys);
figure(4);  
step(closed_loop_kp_sys);

%calculate Kp w.r.t KV = 200;
KV = 200;  
Kp = 134;
controller_sys = Kp + Ki/s + Kd*s; 
open_loop_kp_kv_sys = controller_sys* open_loop_sys;
closed_loop_kp_kv_sys = feedback(open_loop_kp_kv_sys,1); 
figure(5); 
step(closed_loop_kp_kv_sys); 

%calculate the error w.r.t. Kp = 0.0357
%gotta simulate the input and calculate the error 
%lsim(H,u(t),t)
%using the kp value 0.0357
Tfinal = 100; 
dt = 1;
t = 0:dt:Tfinal; 
unitstepFunc = t>=0; 
rampFunc = t.*unitstepFunc; 
quadFunc = t.^2.*unitstepFunc; 

Numerator = cell2mat(closed_loop_kp_sys.Numerator);
Denominator = cell2mat(closed_loop_kp_sys.Denominator); 

%variables c/as^2+bs+c = G(s)
c = Numerator(end);
a = Denominator(1); 
b = Denominator(2); 
if c ~= Denominator(3)
	printf('Error!');	
end
 
Wn = sqrt(c/a); %undamped natural frequency
damping_ratio = b/(2*a*Wn); 
Wd = Wn * sqrt(1 - damping_ratio^2);
y = 1 - exp((t.*(-damping_ratio*Wn))).*(cos(t.*Wd)+(damping_ratio/sqrt(1- damping_ratio^2))*sin(t.*Wd));

errorSignal_position = y - unitstepFunc; 
errorSignal_velocity = y - rampFunc; 
errorSignal_acceleration = y - quadFunc; 

figure(6); 
lsim(closed_loop_kp_sys, unitstepFunc, t);
hold
plot(t, -errorSignal_position, 'r');

figure(7); 
lsim(closed_loop_kp_sys, rampFunc, t); 
hold
plot(t, -errorSignal_velocity, 'r');

figure(8); 
lsim(closed_loop_kp_sys, quadFunc, t); 
hold
plot(t, -errorSignal_acceleration, 'r');  



