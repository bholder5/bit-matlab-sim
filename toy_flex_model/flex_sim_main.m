clc
clear

% Run initialization script
load('flex_model');
%% Setup Simulation
% initial conditions, state is dtheta; theta
x0 = zeros(length(A_hat(:,1)),1);
x0 = [0.250173106775130
  38.155130868579761
 -23.030577377981288
   0.000000112819342
  -0.000000071361371
  -0.000006445470363
  -0.000000026371519
  -0.000002665372205
   0.000001043853094
   0.000000972492952
   0.000002248360013
   0.000000428371111
   0.000001107532036
   0.000000740223294
   0.000000260145986
   0.000000227355235
   0.000000419472119
   0.000000084217297
   0.000000010677850
   0.000000200733451
  -0.000000532313150
   0.000000436498693
  -0.000000301674871
  -0.000000084068741
   0.007497598241569
   1.162360716888447
  -0.694500179909860
  -0.000000001625987
  -0.000000001120601
  -0.000000412167884
  -0.000000002327926
  -0.000000000865266
  -0.000000025174916
   0.000000140257488
   0.000000205906428
  -0.000000005351119
   0.000000013721936
   0.000000006883151
   0.000000002695511
   0.000000002354632
   0.000000004030883
   0.000000000864164
   0.000000000501949
   0.000000002281040
  -0.000000005412931
   0.000000004428654
  -0.000000002668444
  -0.000000000824711];

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
y_enc_d = [0.01,0.01,0.01,-0.01,-0.01]';

y_int = zeros(length(y_enc_d), 1);

Kp = 0*diag([1, 1, 1, 1, 1])/100;
Kd = 0*diag([100, 100, 100, 100, 100])/100;
Ki = 0*diag([10, 10, 10, 10, 10])/100;

%% Sim
while step < length(t_vec)
    step = step + 1;
    if ~mod(t_vec(step), 1000)
        timeis = t_vec(step);
    end
    
    % calculate control torques
%     if ~mod(t_vec(k), 0.02)
        %identity - (C_true' * C_des) and then extract euler angles 5.37
    y_pos = C_hat * x_true;
    y_gyr = C_enc * x_true;

    y_int = y_int + (y_enc_d - y_pos)*dt;

    tau_applied = (Kp * (y_enc_d - y_pos)) + (Kd * -y_gyr) + (Ki * y_int);
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
%     
%     th_over = y_true(10:18) > pi;
%     th_under = y_true(10:18) < -pi;
%     y_true(10:18) = y_true(10:18) + 2*pi*th_under - 2*pi*th_over;
    
    %% save historical data
%     x_all(:,step) = x_true;
    if ~mod(step,100)
        y_all(:,(step/100)+1) = [y_pos; y_gyr];
    end

%     dlmwrite('/media/brad/linux_storage/sim_data/matlab_out.csv',...
%         [dt, y_true'],'delimiter',',', 'precision',15, '-append');
    
    tau(:,step) = tau_applied;
%     e_hist(:,step) = err;

end

% save('no_control_testing.mat')
% Post_processing


plot(t_plot, y_all(1,:))
figure()
plot(t_plot, y_all(2,:))
% figure()
% plot(t_plot, y_all(3,:))
figure()
plot(t_plot, y_all(4,:))
% figure()
% plot(t_plot, y_all(5,:))
figure()
plot(t_plot, y_all(6,:))
legend('yaw gyro')
figure()
% plot(t_plot, y_all(7,:))
% figure()
plot(t_plot, y_all(8,:))
legend('roll gyro')
% figure()
% plot(t_plot, y_all(9,:))
figure()
plot(t_plot, y_all(10,:))
legend('pitch gyro')
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
