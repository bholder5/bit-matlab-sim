clc
clear

% Run initialization script
addpath('~/bit-matlab-sim/Miscellaneous/')
addpath('~/bit-matlab-sim/ADCS/')
addpath('~/bit-matlab-sim/Plant_functions/')
addpath('/home/brad/bit-matlab-sim/')
% load('/home/brad/bit-matlab-sim/flexible_model_data/flex_model');
%% Setup Simulation
% initial conditions, state is dtheta; theta
[ndof, g0, r_n1_n, z_n, p_n, m_n, c_n, ...
    i_n, m_w_n,  i_rw, bear_k_cst, bear_c_cst, k_d, b_d, ...
    w_rw_max, w_rw_nom, hs_rw, hs_rw_max, a_flex, b_flex, a_df, b_df] = init_func();

x_flex0 = zeros(110,1);
y_flex = x_flex0;

%% Add rigid modes into a_flex and b_fleex for sim
a_rig = [zeros(3,3), eye(3); zeros(3,6)];
m_rig = diag([991.0, 364.0, 143.0]);
b_in = [1 0 0 0 0; 0 1 1 0 0; 0 0 0 1 1];
b_rig = [zeros(3,5); inv(m_rig)*b_in ];

A_sys = zeros(110, 110);
B_sys = zeros(110,5);

A_sys(1:6, 1:6) = a_rig;
A_sys(7:110, 7:110) = a_df;

B_sys(1:6, :) = b_rig;
B_sys(7:110, :) = b_df;

tau_app_flex = zeros(3,1);

%% Build our system for control, aka dont include rigid position

a_ctrl = zeros(107,107);
b_ctrl = zeros(107,5);

a_ctrl(4:107,4:107) = a_df;
b_ctrl = B_sys(4:110, :);

%% calculate controol based on passive control model from lecture 7a of damaren course
num_modes = 4;
num_states = 3+2*num_modes;
a_use = a_ctrl(1:num_states, 1:num_states);
b_use = b_ctrl(1:num_states, :);
R = 0.5*eye(5);
gn = 1:num_modes*2;
gns = 1*[0.001, 0.001, 0.001, gn]
% QR = 1*eye(size(a_use));
QR = diag(gns)
QL = 1*eye(size(a_use));

QR(1:3, 1:3) = QR(1:3, 1:3) * 0.00001
QL(1:3, 1:3) = QL(1:3, 1:3) * 0.00001

%% override to only use flexible modes
a_use = a_df(1:num_modes*2, 1:num_modes*2);
b_use = b_df(1:num_modes*2, :);

QR = 1*eye(size(a_use));
QL = eye(size(a_use));

[ac, bc, cc] = flex_ctrl(a_use, b_use, b_use', R, QR, QL);

x_ctrl = zeros(num_states,1);

y_flex = zeros(104,1);
x_ctrl = zeros(num_modes*2,1);
%% initiate flex system
sys_flex = @(y_flex, tau_app_flex, tau_flex) flex_propogate(a_df, b_df, tau_app_flex, tau_flex, y_flex);
sys_ctrl = @(x_ctrl, gyros) ac*x_ctrl + bc*gyros;
% Sim Parameters
t0 = 0;
tf = 60 ;
dt = 1e-3;
t_vec = 0:dt:tf;
t_plot = 0:dt*100:tf;

y_all = zeros(104, length(t_plot));

step = 0;


tau_flex = zeros(5,1)+10;

%% Sim
while step < length(t_vec)
    step = step + 1;
%     if ~mod(t_vec(step), 1000)
%         timeis = t_vec(step);
%     end
%     
%     if t_vec(step) > 2
%         tau_applied = zeros(5,1);
%     end

    % tau_app_flex = tau_applied(7:9);
    % tau_applied(7) = tau_applied(7) + tau_flex(1);
    % tau_applied(8) = tau_applied(8) + tau_flex(2) + tau_flex(3);
    % tau_applied(9) = tau_applied(9) + tau_flex(4) + tau_flex(5);
    % 
    if step < 100
        tau_flex = zeros(5,1)+10;
    end
    
    % if step > 100
    %     tau_flex = zeros(5,1);
    % end

    %% Propagate the system 
    %RK4 solver
%% Propogate flexible system
    kf1 = sys_flex(y_flex, tau_app_flex, tau_flex) * dt;
    kf2 = sys_flex(y_flex + (kf1/2), tau_app_flex, tau_flex) * dt;
    kf3 = sys_flex(y_flex + (kf2/2), tau_app_flex, tau_flex) * dt;
    kf4 = sys_flex(y_flex + kf3, tau_app_flex, tau_flex) * dt;

    eta_dd = ((kf1+(2*kf2)+(2*kf3)+kf4)/6);
    y_flex = y_flex + eta_dd;  
    
    gyros = b_df' * y_flex;

    %% propogate control state
    kc1 = sys_ctrl(x_ctrl, gyros) * dt;
    kc2 = sys_ctrl(x_ctrl + (kc1/2), gyros) * dt;
    kc3 = sys_ctrl(x_ctrl + (kc2/2), gyros) * dt;
    kc4 = sys_ctrl(x_ctrl + kc3, gyros) * dt;

    ctrl_dd = ((kc1+(2*kc2)+(2*kc3)+kc4)/6);
    x_ctrl = x_ctrl + ctrl_dd; 

    tau_flex = cc * x_ctrl;

    %% save historical data
    % x_all(:,step) = x_true;
    if ~mod(step,100)
        y_all(:,(step/100)+1) = [y_flex];
    end

end
tau_flex
% save('no_control_testing.mat')
% % Post_processing
% % 
% % 
% % plot(t_plot, y_all(1,:))
% figure(1)
% subplot(3,2,1)
% hold on
% % plot(t_plot, y_all(2,:))
% % figure()
% % plot(t_plot, y_all(3,:))
% % figure()
% % plot(t_plot, y_all(4,:))
% % figure()
% % plot(t_plot, y_all(5,:))
% % figure()
% % plot(t_plot, y_all(6,:))
% % legend('yaw gyro')
% % figure()
% plot(t_plot, y_all(7,:))
% % figure(2)
% subplot(3,2,2)
% hold on
% plot(t_plot, y_all(9,:))
% % legend('roll gyro')
% % figure(3)
% subplot(3,2,3)
% hold on
% plot(t_plot, y_all(11,:))
% % figure(4)
% subplot(3,2,4)
% hold on
% plot(t_plot, y_all(13,:))
% % figure(5)
% subplot(3,2,5)
% hold on
% plot(t_plot, y_all(15,:))
% % figure(6)
% subplot(3,2,6)
% hold on
% plot(t_plot, y_all(17,:))

figure(1)
subplot(3,2,1)
hold on
% plot(t_plot, y_all(2,:))
% figure()
% plot(t_plot, y_all(3,:))
% figure()
% plot(t_plot, y_all(4,:))
% figure()
% plot(t_plot, y_all(5,:))
% figure()
% plot(t_plot, y_all(6,:))
% legend('yaw gyro')
% figure()
plot(t_plot, y_all(1,:))
% figure(2)
subplot(3,2,2)
hold on
plot(t_plot, y_all(2,:))
% legend('roll gyro')
% figure(3)
subplot(3,2,3)
hold on
plot(t_plot, y_all(3,:))
% figure(4)
subplot(3,2,4)
hold on
plot(t_plot, y_all(4,:))
% figure(5)
subplot(3,2,5)
hold on
plot(t_plot, y_all(5,:))
% figure(6)
subplot(3,2,6)
hold on
plot(t_plot, y_all(6,:))

