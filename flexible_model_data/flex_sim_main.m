clc
clear

% Run initialization script
load('/home/brad/bit-matlab-sim/flexible_model_data/flex_model');
%% Setup Simulation
% initial conditions, state is dtheta; theta
x0 = zeros(length(A_hat(:,1)),1);


x_true = x0;

% Sim Parameters
t0 = 0;
tf = 10 ;
dt = 1e-4
t_vec = 0:dt:tf;
t_plot = 0:dt*100:tf;

y_all = zeros(10, length(t_plot));

step = 0;

tau_applied = zeros(length(B_hat(1,:)),1);
tau = zeros(length(B_hat(1,:)),length(t_vec));

%took out t from (t, y_true) for rk4
sys = @(x_true, tau_applied) ((A_hat * x_true) + (B_hat * tau_applied))

tau_applied = zeros(5,1)+10;

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
    
    %% Propagate the system 
    %RK4 solver

    k1 = sys(x_true, tau_applied) * dt;
    k2 = sys(x_true + (k1/2), tau_applied) * dt;
    k3 = sys(x_true + (k2/2), tau_applied) * dt;
    k4 = sys(x_true + k3, tau_applied) * dt;
% 
    tdd = ((k1+(2*k2)+(2*k3)+k4)/6);
    x_true = x_true + tdd;   
    
%     y_gyro1 = C_gyro1 * x_true;
%     y_gyro2 = C_gyro1 * x_true;

    %% save historical data
%     x_all(:,step) = x_true;
%     if ~mod(step,100)
%         y_all(:,(step/100)+1) = [y_gyro1; y_gyro2; x_true(4:7)];
%     end

end

% save('no_control_testing.mat')
% Post_processing
% 
% 
% plot(t_plot, y_all(1,:))
% figure()
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
% plot(t_plot, y_all(7,:))
% figure()
% plot(t_plot, y_all(8,:))
% legend('roll gyro')
% figure()
% plot(t_plot, y_all(9,:))
% figure()
% plot(t_plot, y_all(10,:))
% legend('pitch gyro')
% figure()
% plot(t_vec, tau(1,:))
% figure()
% plot(t_vec, tau(2,:))
% figure()
% plot(t_vec, tau(3,:))
% figure()
% plot(t_vec, tau(4,:))
% figure()
% plot(t_vec, tau(5,:))
