addpath('~/bit-matlab-sim/Miscellaneous/')
addpath('~/bit-matlab-sim/ADCS/')
addpath('~/bit-matlab-sim/Plant_functions/')
addpath('~/bit-matlab-sim/flexible_model_data/')
% addpath('/home/brad/bit-matlab-sim/codegen/mex/bit_one_step_mex')

ARCSEC = 4.848136805555555555555555555e-6;
ARCMIN = inv(180/pi*60);
tau_f_max = 11.14; 
ts_f = 21.965/7.718/1000.0; 
tau_f_thresh = 0.001;
tau_c_max = 11.14;

factor = 15;

kp_f = 10*[1000,10,10]';
kd_f = [0.001,0.00,0.001]';
ki_f = [0.1,0.1,1]';

kp_rw = 18.0;
kd_rw =  0.;
ki_rw =  1.8;

q_k = eye(3) * ((ARCSEC/3.0/(sqrt(50))*4.0)^2);
r_k = eye(3) * ((ARCSEC/3.0/5.0)^2);

% theta_0 = 0*[0.,0.4*pi/180.0,0.4*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.,0.1,40*pi/180]';
theta_0 = 0*[0,0,0,0,0,0.0,0,0,-40*pi/180]';
thet_pit_nom = 0* -40*pi/180

%setting IC from past sim
% y0 = [0.017552353814854, -0.002156992032555, -0.002273627285241, ...
%     -0.004091940730352,  -0.002796089196615,   0.019674817779806,...
%     -0.017606183923045,                   0,                   0, ...
%      0.207860712172010,  -0.003878840466313,  -0.004340266988222, ...
%     -0.001098037684871,  -0.001085183886166,  -0.001924742862772, ...
%      2.937417436471931,                   0,                   0, ...
%                      0,                   0, 28.274274172758336]';
theta_des = double([0.,0.4*pi/180.0,0.4*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.,0,-40*pi/180]');
d_theta_dt_0 = double([0.,0.,0.,0.,0.,0.,0.,0.,0.]'); 

y_flex = double(zeros(104,1)+1.1);
x_flex0 = y_flex;

y_flex2 = double(zeros(104,1)+1.1);
x_flex20 = y_flex;

tau_flex = double(zeros(5,1));


i_rw = double(reshape([10.0,0.,0.,0.,10.0,0.,0.,0.,9.0], 3, 3));
hs_rw = double(i_rw*pi*[0.0; 0.0; 1.0]);

x0 = double([d_theta_dt_0; theta_0; hs_rw]);
x20 = double([d_theta_dt_0; theta_0; hs_rw]);
z_n = double([0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
       0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
       1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]);

dt = double([1e-3, 1e-4]);
num_steps = 10;
t = double(0:dt(1):dt(1)*10);
w_piv = double(0.01);
piv_flag = false;
unlock1 = double([1,0,0,0,0,0,0,0,0]');
unlock2 = double([1,1,1,1,1,1,1,1,1]');
unlock = eye(9);
tau_applied = double(0.01*[10000; -1000; 1000; -1000; 1000; -1000; 0; 0; 0]);
tau_max_piv = double(20);
% tau_applied = [10;0;0;0;0;0;0;0;0]+0.1;
%% run propagator loop
x0 = double([d_theta_dt_0; theta_0; hs_rw]);
for step = 1:length(t)
    [y_all, y_flex] = bit_one_step(double(x0),  double(tau_applied), double(unlock2), double(w_piv), uint8(false), ...
        double(dt(1)), uint16(num_steps), double(tau_max_piv), double(thet_pit_nom), double(x_flex0), double(tau_flex), uint8(false), uint8(true))
    x0 = y_all(:);
    x_flex0 = y_flex;
end

for step = 1:length(t)
    [y_all2, y_flex2] = bit_one_step(double(x20),  double(tau_applied), double(unlock2), double(w_piv), uint8(false), ...
        double(dt(1)), uint16(num_steps), double(tau_max_piv), double(thet_pit_nom), double(x_flex0), double(tau_flex), uint8(false), uint8(false))
    x20 = y_all2(:);
    x_flex20 = y_flex2;
end
    %     y_all2 = bit_one_step(x0, tau_applied, unlock(:,step), w_piv, true, dt(mod(step,2)+1), uint16(5));
for step = 1:5
    theta_des = theta_des*0.8;
    x0 = x0*1.01;
    
    C = compute_rotation_mat_C(double(z_n), double(theta_des));
    Cr = compute_rotation_mat_roll_C(double(z_n), double(theta_des));
    Cy = compute_rotation_mat_yaw_C(double(z_n), double(theta_des));
    [ax, rot] = rot2axis_C(double(C))
    omegay = compute_angular_velocity_yaw_C(double(x0(1:18)), double(z_n));
    omegar = compute_angular_velocity_roll_C(double(x0(1:18)), double(z_n));
    omega = compute_angular_velocity_C(double(x0(1:18)), double(z_n));
end




















