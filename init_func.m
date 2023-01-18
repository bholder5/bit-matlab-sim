function [ndof, g0, r_n1_n, z_n, p_n, m_n, c_n, ...
    i_n, m_w_n,  i_rw, bear_k_cst, bear_c_cst, k_d, b_d, ...
    w_rw_max, w_rw_nom, hs_rw, hs_rw_max] = init_func()
    %%% This file initializes are parameters for simulating BIT (9 DOF)
    %%% This file initializes are parameters for simulating BIT (9 DOF)
ndof = 9;
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

m_n = [0.0,0.0,100000.0,0.0,0.0,1.0,350.0,73.0,150.0];

%each column is vector (COM of B3 (flight train) is 30.5m along z
c_n = [0.,0.,  0.0,0.,0., 0.0,0.,0.,0.;
       0.,0.,  0.0,0.,0., 0.0,0.,0.,0.;
       0.,0.,-30.5,0.,0.,-1.4,0.,0.,0.];
%each row is row wise matric
i_n = [ [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0],
        [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0],
        [5448000.0,  0.0,  0.0,  0.0, 5448000.0,  0.0,  0.0,  0.0, 5448000.0],
        [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0],
        [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0],
        [  1.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,   1.0],
        [  395,    0,    0,    0,  378,    0,    0,    0,   304],
        [ 10.5,  0.0,  0.0,  0.0, 19.2,  0.0,  0.0,  0.0,  25],
        [   22,  0.0,  0.0,  0.0, 50.0,  0.0, 0, 0, 50]];
    
% calculate the mass matricies
m_w_n = zeros(6,6,9);
for k = 1:9
    mass = eye(3) * m_n(k);
    i_k = reshape(i_n(k,:), 3, 3);
    offterm = xmat(c_n(:,k));
           
    m_w_n_i = [mass, -offterm; offterm, i_k];
    m_w_n(:,:,k) = m_w_n_i;
end

i_rw = reshape([2.5,0.,0.,0.,2.5,0.,0.,0.,4.5], 3, 3);
bear_k_cst = 0.0*2.0*0.9486*(0.0254*4.44822162)*180.0/pi;
bear_c_cst = 0.; 
    
k_d = [0.,0.,0.,0.,0.,0.1*pi/180.0,0.,bear_k_cst,bear_k_cst]';
b_d = [0.,0.,0.,0.,0.,0.,0.,0.,0.]';

w_rw_max = 2.0*pi;
w_rw_nom = pi; 
hs_rw = i_rw * w_rw_nom * z_n(:,7);
hs_rw_max = i_rw * w_rw_max * z_n(:,7);


% theta_0 = [0.,0.4*pi/180.0,0.4*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.1*pi/180.0,0.,0.1,-40*pi/180]';
theta_0 = [0,0,0,0,0,0,0,0,0,0]';

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

unlock = [1.0,1.0,1.0,1.0,1.0,1.0,1,1,1]';
latency = 0.0150;
fs =   50.0; 
fs_ekf = 1000.0;






end

