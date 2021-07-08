clc

addpath('C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Miscellaneous', ...
    'C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Plant_functions', ... 
    'C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Control'); 

load('C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\no_control_for_estimator.mat')

% Assume calibration encorporating misalignment and gain are known aka
% measurements are in frame

%psi = angular velocity measured in Fb
%PSI = rotation for time step tk-1 -> tk

%y^=[dtheta; theta; bias; ]
y_hat_cur = zeros(19,1);
dt_ekf = 1/fs_ekf;
dt_factor = dt_ekf/dt;

% KVH characteristics
% (https://sites.physics.utoronto.ca/bit/documentation/electronics/data-sheets/dsp-1750-dig-1axis-std-manual.pdf)
kvh_lat = 1.3/1000;
kvh_bias_rate = ((0.05 * pi / 180)/(3600)) * dt_ekf;
kvh_bias_offset = 1*(10 * pi / 180)/3600 * dt_ekf;
kvh_rndwlk = (0.8 * pi /180)/sqrt(1000) * dt_ekf;

dc_b = zeros(3,1) + kvh_bias_offset;

C_k0_hat = compute_rotation_mat(z_n, y_all(10:18,1));
C_k1 = compute_rotation_mat(z_n, y_all(10:18,2));

num_steps = length(e_hist(1,:))/dt_factor-1;
err = zeros(3,num_steps);
phi_hat = err;
phi = err;
%Build bias H matricies, here the first 3,3 block is PSI_k but everyting
%else is constant
Hx_bias = [zeros(3,3),diag(dt_ekf, dt_ekf, dt_ekf); zeros(3,3), eye(3)];
Hw_bias = [diag(dt_ekf, dt_ekf, dt_ekf), zeros(3,3); zeros(3,3), eye(3)];


for step = 1:num_steps
    
    % set k so ekf works at proper frequency
    k = (step-1) * dt_factor + 1;
    
    if mod(step,10) ~= 0
        
        %assemble rate gyro measurements including DC bias
        w_true = compute_angular_velocity(y_all(10:18, k), y_all(1:9, k), z_n) %Pointing rate
        dc_b = dc_b + (kvh_bias_rate*(-1 + 2*rand(3,1))) % bias changes as random walk with max value 
        gyro_noise = (kvh_rndwlk*(-1 + 2*rand(3,1)));
        w_meas = w_true + gyro_noise + dc_b;

        %Calculate the change over the time step 2.15
        psi_k = w_meas * dt_ekf ;
        mag_psi_k = norm(psi_k);
        PSI_k = axis2rot(psi_k/mag_psi_k,mag_psi_k);

        %2.14 telescope propagation from gyros

        C_k1_hat = PSI_k * C_k0_hat;
        C_k1 = compute_rotation_mat(z_n,y_all(10:18,k+dt_factor));

        phi_k1_hat = parameterize_312_rotation(C_k1_hat);
        phi_t = parameterize_312_rotation(C_k1);
        delta_C = unxmat(eye(3) - (C_k1_hat * C_k1'))/ARCSEC;
        
        %Build bias H matricies
        Hx_bias(1:3,1:3) = PSI_k;
        
    else
        delta_C = zeros(3,1)
        C_k1 = compute_rotation_mat(z_n,y_all(10:18,k+dt_factor));
        phi_t = parameterize_312_rotation(C_k1);
        phi_k1_hat = phi_t;
        C_k1_hat = C_k1
    end
    %store results
    err(:,step) = delta_C;
    phi_hat(:,step) = phi_k1_hat;
    phi(:,step) = phi_t;
    
    %reassign k0 state for next iter
    C_k0_hat = C_k1_hat;
    
end

t=dt_ekf:dt_ekf:dt_ekf*num_steps;


figure(1)
tiledlayout(4,1)
nexttile
plot(t,err(1,:),t,err(2,:),t,err(3,:));
legend('roll','pitch', 'yaw')
title('Error')
ylabel('ARCSEC')

nexttile
plot(t,phi_hat(1,:), t,phi(1,:));
legend('\phi^\^', '\phi')
title('Roll')

nexttile
plot(t,phi_hat(2,:),t,phi(2,:));
legend('\phi^\^', '\phi')
title('pitch')

nexttile
plot(t,phi_hat(3,:),t,phi(3,:));
legend('\phi^\^', '\phi')
title('Yaw')





