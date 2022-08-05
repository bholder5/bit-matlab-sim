clc
clear


% addpath('C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Miscellaneous', ...
%     'C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Plant_functions', ... 
%     'C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Control'); 
addpath('~/bit-matlab-sim/Miscellaneous/')
addpath('~/bit-matlab-sim/ADCS/')
addpath('~/bit-matlab-sim/Plant_functions/')
addpath('~/bit-matlab-sim/codegen/mex/bit_one_step_mex/')

% Run initialization script
Initialization;
%% Setup Simulation
% initial conditions, state is dtheta; theta
x0 = [d_theta_dt_0; theta_0; hs_rw];
% x0 = y0;
y_true = x0;
y_mex = x0;
buffer = y_true;
y_lat = zeros(21,1);

% Sim Parameters
t = 0;
t1 = 1;
dt1 = 1e-3;
tf = 30 ;
dt = 1e-3
t_vec1 = 0:dt1:t1;
t_vec2 = t1+dt:dt:tf;
t_vec  = [t_vec1, t_vec2];
m = 10; %num terms used for fit


% y_all1 = zeros(18, tf/(dt));
% y_all = zeros(21, length(t_vec));
% y_all(:,1) = x0;
% y_all_mex = y_all;
% tau = zeros(9, length(t_vec));
step = 1;

% simulation vectors
C_true = compute_rotation_mat(z_n, x0(10:18));
phi_true = parameterize_312_rotation(C_true); %Pointing angle
C_des = compute_rotation_mat(z_n, theta_des);    %Desired Pointing angle
w_true = zeros(3,1); %Pointing rate
% tau_applied = zeros(9,1);

piv_flag = true;
w_piv = 0.01;
tau_max_piv = 20;
thet_pit_nom = deg2rad(-40);

%took out t from (t, y_true) for rk4
sys = @(y_true, tau_applied, dw_piv) bit_propagator(y_true, c_n, z_n, m_n, r_n1_n, m_w_n, p_n, ... 
    k_d, b_d, g0, unlock, hs_rw_max, tau_applied, w_piv, piv_flag, dw_piv, tau_max_piv, thet_pit_nom)

err_sum = zeros(3,1);
err = zeros(3,1);
e_hist = zeros(3,length(t_vec));

err_tc = 10;
err_decay = exp(-dt/err_tc);
%% Sim
tau_applied = zeros(9,1);

% table = array2table([dt, y_true']);
% table.Properties.VariableNames = {'dt', 'dth1', 'dth2','dth3','dth4','dth5',...
%     'dth6','dth7','dth8','dth9','th1','th2','th3','th4','th5','th6','th7',...
%     'th8','th9', 'h1', 'h2', 'h3'};
% writetable(table, '/media/brad/linux_storage/sim_data/matlab_out.csv')

while step < length(t_vec)
    step = step + 1;
%     dt = t_vec(step) - t_vec(step-1);
    if ~mod(t_vec(step), 1000)
        timeis = t_vec(step)
    end
    %should be able to add balloon velocity as noise factor and drift in
    %it, bArth will get me this.
    
    %% calculate control torques
%     if ~mod(t_vec(k), 0.02)
%         %identity - (C_true' * C_des) and then extract euler angles 5.37
%         C_true = compute_rotation_mat(z_n, y_true(10:18));
%         [omega_true] = compute_angular_velocity(y_true(10:18), y_true(1:9), z_n); %Pointing rate
%         err = unxmat(eye(3) - (C_true * C_des'));
%         err_sum = (err_sum .* err_decay) + (err*dt);
%         tau_applied = PID_control(err, omega_true, y_true(16:18), y_true(7:9),...
%             z_n, err_sum, kp_f, kd_f,ki_f, kp_rw, kd_rw, ki_rw, tau_f_max);
%     %     tau_applied = zeros(9,1);

%         w_piv = rw_g1*((-hs(3)/i_rw(3,3))-w_rw_nom) - (rw_g2*tau_rw);
%     end
    %% Propagate the system
    % takes in system dynamics and applied torques
    %[Y, T] = rkf45_1(sys, 0, dt, y_true, dt/10, h);
   
    %RK4 solver

%     k1 = sys(y_true, tau_applied) * dt;
%     k2 = sys(y_true + (k1/2), tau_applied) * dt;
%     k3 = sys(y_true + (k2/2), tau_applied) * dt;
%     k4 = sys(y_true + k3, tau_applied) * dt;
% 
%     tdd = ((k1+(2*k2)+(2*k3)+k4)/6);
%     y_true = y_true + tdd;   
%     
%     th_over = y_true(10:18) > pi;
%     th_under = y_true(10:18) < -pi;
%     y_true(10:18) = y_true(10:18) + 2*pi*th_under - 2*pi*th_over;
%     
    %% save historical data
%     y_true = bit_one_step_mex('bit_one_step',y_true, tau_applied, unlock, w_piv, piv_flag, dt/10, uint16(10));
    dw_piv = (w_piv - y_true(6))/dt
    
    y_true = bit_one_step(y_true, tau_applied, unlock, w_piv, piv_flag, ...
    dt, uint16(10), tau_max_piv, thet_pit_nom);
    C = compute_rotation_mat(z_n, y_true(10:18));
    omega = compute_angular_velocity_C(y_true(1:18), z_n);
    %MEX File check
%     y_mex = bit_one_step_mex('bit_one_step',y_mex, tau_applied, unlock, ...
%         w_piv, piv_flag, dt, uint16(10), tau_max_piv);
    
    %% save historical data
%     y_all(:,step) = y_true;
%     y_all_mex(:,step) = y_mex;
%     diff = y_true - y_mex
%     dlmwrite('/media/brad/linux_storage/sim_data/matlab_out.csv',...
%         [dt, y_true'],'delimiter',',', 'precision',15, '-append');
    
%     tau(:,step) = tau_applied;
%     e_hist(:,step) = err;
    diff = y_true-y_mex
end

save('no_control_testing.mat')
% Post_processing
