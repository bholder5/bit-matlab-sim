%%% This file initializes are parameters for simulating BIT (9 DOF)
ndof = 9;
ARCSEC = 4.848136805555555555555555555e-6;
ARCMIN = inv(180/pi*60);

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
%ballon is based on 5000lb of palstic at 120m diameter
i_n = [ [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0],
        [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0],
        [5448000.0,  0.0,  0.0,  0.0, 5448000.0,  0.0,  0.0,  0.0, 5448000.0],
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
hs_rw_max = i_rw * w_rw_max * z_n(:,7);

tau_f_max = 11.14; 
ts_f = 21.965/7.718/1000.0; 
tau_f_thresh = 0.001;
tau_c_max = 11.14;

factor = 15;

kp_f = 10*[1000,10,10]';
kd_f = [0.001,0.00,0.001]';
ki_f = [0.1,0.1,1]';

rw_g1 = 0.034;
rw_g2 = 0.8* (2*sqrt(rw_g1/(i_rw(3,3)*k_d(6))));

kp_rw = 18.0;
kd_rw =  0.;
ki_rw =  1.8;

q_k = eye(3) * ((ARCSEC/3.0/(sqrt(50))*4.0)^2);
r_k = eye(3) * ((ARCSEC/3.0/5.0)^2);

theta_0 = [0.,0.4*pi/180.0,0.4*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.,0.1,40*pi/180]';
% theta_0 = [0.,0.4*pi/180.0,0.4*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.,0,0]';

%setting IC from past sim
% y0 = [0.017552353814854, -0.002156992032555, -0.002273627285241, ...
%     -0.004091940730352,  -0.002796089196615,   0.019674817779806,...
%     -0.017606183923045,                   0,                   0, ...
%      0.207860712172010,  -0.003878840466313,  -0.004340266988222, ...
%     -0.001098037684871,  -0.001085183886166,  -0.001924742862772, ...
%      2.937417436471931,                   0,                   0, ...
%                      0,                   0, 28.274274172758336]';
theta_des = [0.,0.4*pi/180.0,0.4*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.,0.1,40*pi/180]';
d_theta_dt_0 = [0.,0.,0.,0.,0.,0.,0.,0.,0.]'; 

unlock = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]';
latency = 0.0150;
fs =   50.0; 
fs_ekf = 1000.0;


