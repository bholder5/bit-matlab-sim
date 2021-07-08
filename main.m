clc
clear


addpath('C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Miscellaneous', ...
    'C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Plant_functions', ... 
    'C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Control'); 
% Run initialization script
Initialization;
%% Setup Simulation
% initial conditions, state is dtheta; theta
x0 = [d_theta_dt_0; theta_0; hs_rw];
% x0 = y0;
y_true = x0;
buffer = y_true;
y_lat = zeros(21,1);

% Sim Parameters
t = 0;
tf = 600;
dt = 2e-2;
m = 10; %num terms used for fit


% y_all1 = zeros(18, tf/(dt));
y_all = zeros(21, tf/(dt));
tau = zeros(9, tf/(dt));
step = 0;

% simulation vectors
C_true = compute_rotation_mat(z_n, x0(10:18));
phi_true = parameterize_312_rotation(C_true); %Pointing angle
C_des = compute_rotation_mat(z_n, theta_des);    %Desired Pointing angle
w_true = zeros(3,1); %Pointing rate
tau_applied = zeros(9,1);

%took out t from (t, y_true) for rk4
sys = @(y_true, tau_applied) bit_propagator(y_true, c_n, z_n, m_n, r_n1_n, ...
    m_w_n, p_n, k_d, b_d, g0, i_rw, unlock, hs_rw_max, rw_g1, rw_g2, ...
    w_rw_nom, tau_applied);

err_sum = zeros(3,1);
err = zeros(3,1);
e_hist = zeros(3,tf/dt);

err_tc = 10;
err_decay = exp(-dt/err_tc);
%% Sim
while step < tf/dt
    step = step + 1;   
    
%     if step ==  100000
%         kp_f = [0,0,60]';
%         kd_f = [0.0,0.0,0.3]';
%         ki_f = [0.0,0.0,0.1]';
%     end
%     if step ==  100000 + (15000 * 2)
%         step
%         kp_f = [0,0,60]';
%         kd_f = [0.0,0.0,0.4]'
%         ki_f = [0.0,0.0,0.1]';
%     end
%     if step ==  100000 + (15000 * 3)
%         step
%         kp_f = [0,0,60]';
%         kd_f = [0.0,0.0,0.5]'
%         ki_f = [0.0,0.0,0.1]';
%     end
%     if step ==  100000 + (15000 * 4)
%         kp_f = [0,0,60]';
%         kd_f = [0.0,0.0,0.6]';
%         ki_f = [0.0,0.0,0.1]';
%     end
    %should be able to add balloon velocity as noise factor and drift in
    %it, bArth will get me this.
    
    %% calculate control torques
    %identity - (C_true' * C_des) and then extract euler angles 5.37
    C_true = compute_rotation_mat(z_n, y_true(10:18));
    [omega_true] = compute_angular_velocity(y_true(10:18), y_true(1:9), z_n); %Pointing rate
    err = unxmat(eye(3) - (C_true * C_des'));
    err_sum = (err_sum .* err_decay) + (err*dt);
    if step == 5000
        step
    end
    tau_applied = PID_control(err, omega_true, y_true(16:18), y_true(7:9),...
        z_n, err_sum, kp_f, kd_f,ki_f, kp_rw, kd_rw, ki_rw, tau_f_max)
%     tau_applied = zeros(9,1);
    
    %% Propagate the system
    % takes in system dynamics and applied torques
    %[Y, T] = rkf45_1(sys, 0, dt, y_true, dt/10, h);
   
    %RK4 solver

    k1 = sys(y_true, tau_applied) * dt;
    k2 = sys(y_true + k1/2, tau_applied) * dt;
    k3 = sys(y_true + k2/2, tau_applied) * dt;
    k4 = sys(y_true + k3, tau_applied) * dt;

    tdd = ((k1+2*k2+2*k3+k4)/6);
    y_true = y_true + tdd;   
    
    th_over = y_true(10:18) > pi;
    th_under = y_true(10:18) < -pi;
    y_true(10:18) = y_true(10:18) + 2*pi*th_under - 2*pi*th_over;
    
    %% save historical data
    y_all(:,step) = y_true;
    tau(:,step) = tau_applied;
    e_hist(:,step) = err;
end


Post_processing
