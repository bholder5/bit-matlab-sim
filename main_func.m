function [] = main_func()
% addpath('C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Miscellaneous', 'C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Plant_functions'); 
% Run initialization script
%% Initialization;
%%% This file initializes are parameters for simulating BIT (9 DOF)
ndof = 9;
ARCSEC = 4.848136805555555555555555555e-6;

g0 = [0; 0; -9.72];
tel_offset = [0; 40.0*pi/180; 0];

%each column is vector (F4 is -61m from F3)
r_n1_n = [0,0,0,  0,0,0,0,   0,0;
          0,0,0,  0,0,0,0,   0,0;
          0,0,0,-61,0,0,0,-1.4,0];
%each column is vector
z_n = [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
       0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
       1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0];
   
p_n = [zeros(3,9); z_n];

m_n = [0.0,0.0,10.0,0.0,0.0,1.0,350.0,73.0,150.0];

%each column is vector (COM of B3 (flight train) is 30.5m along z
c_n = [0.,0.,  0.0,0.,0., 0.0,0.,0.,0.;
       0.,0.,  0.0,0.,0., 0.0,0.,0.,0.;
       0.,0.,-30.5,0.,0.,-1.4,0.,0.,0.];
%each row is row wise matric
i_n = [ [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0],
        [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0],
        [  1.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  10.0],
        [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0],
        [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0],
        [  1.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,   1.0],
        [150.35,-0.32,-0.07,-0.32,79.89, 0.85,-0.07, 0.85,117.0],
        [ 30.09, 0.0,  0.0,  0.0, 23.17, 0.0,  0.0,  0.0,  19.8],
        [ 33.71, 0.0,  0.0,  0.0, 25.53,10.5,  0.0, 10.5,  22.79]];
    
% calculate the mass matricies
m_w_n = zeros(6,6,9);
for k = 1:9
    mass = eye(3) * m_n(k);
    i_k = reshape(i_n(k,:), 3, 3);
    offterm = xmat(c_n(:,k));
           
    m_w_n_i = [mass, -offterm; offterm, i_k];
    m_w_n(:,:,k) = m_w_n_i;
end

i_rw = reshape([10.0,0.,0.,0.,10.0,0.,0.,0.,9.0], 3, 3);
bear_k_cst = 2.0*0.9486*(0.0254*4.44822162)*180.0/pi;
bear_c_cst = 0.; 
    
k_d = [0.,0.,0.,0.,0.,0.1*pi/180.0,0.,bear_k_cst,bear_k_cst]';
b_d = [0.,0.,0.,0.,0.,0.,0.,0.,0.]';

w_rw_max = 2.0*pi;
w_rw_nom = pi; 
hs_rw = i_rw * w_rw_nom * z_n(:,7);

tau_f_max = 11.14; 
ts_f = 21.965/7.718/1000.0; 
tau_f_thresh = 0.001;
tau_c_max = 11.14;

kp_f = [9000.0,5000.0,12000.0]';
kd_f = [70.0,70.0,70.0]';
ki_f = [60.0,60.0,0.]';

kp_rw = 18.0;
kd_rw =  0.;
ki_rw =  1.8;

q_k = eye(3) * ((ARCSEC/3.0/(sqrt(50))*4.0)^2);
r_k = eye(3) * ((ARCSEC/3.0/5.0)^2);

theta_0 = [0.,0.4*pi/180.0,0.4*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.,0.,0.]';
d_theta_dt_0 = [0.,0.,0.,0.,0.,0.,0.,0.,0.]'; 

unlock = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0]';
latency = 0.0150;
fs =   50.0; 
fs_ekf = 1000.0;

%% 
% initial conditions, state is dtheta; theta
x0 = [d_theta_dt_0; theta_0; hs_rw];
y_true = x0;
buffer = y_true;
y_lat = zeros(21,1);

% Sim Parameters
t = 0;
tf = 2;
h = 1e-9;
m = 10; %num terms used for fit

dt = 1/fs_ekf;

% y_all1 = zeros(18, tf/(dt));
y_all = zeros(21, tf/(dt));
step = 0;

% simulation vectors
phi_true = zeros(3,1); %Pointing angle
phi_des = phi_true;    %Desired Pointing angle
w_true = zeros(3,1); %Pointing rate

sys = @(t, y_true) bit_propagator(y_true, c_n, z_n, m_n, r_n1_n, m_w_n, p_n, k_d, b_d, g0, i_rw, unlock);


while step <= tf/dt
    step = step + 1;
    
    [Y, T] = rkf45_1(sys, 0, dt, y_true, dt/10, h);
    
    y_true = Y(length(T), :)';
    y_all(:,step) = y_true;
    
end

% t = 0:dt:tf;
% plot(t,y_all(10,:),t, y_all(11,:),t,y_all(12,:),t,y_all(13,:),t,y_all(14,:),t, ...
%     y_all(15,:),t,y_all(16,:),t,y_all(17,:),t,y_all(18,:));

% legend('1','2','3','4','5','6','7','8','9')
end

