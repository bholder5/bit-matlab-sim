clc
clear

% addpath('C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Miscellaneous', ...
%     'C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Plant_functions', ... 
%     'C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\Control'); 

% load('C:\Users\General_Grievous\Documents\Brad School\Matlab_BIT_Sim\no_control_for_estimator.mat')

addpath('~/bit-matlab-sim/Miscellaneous/')
addpath('~/bit-matlab-sim/ADCS/')
addpath('~/bit-matlab-sim/Plant_functions/')
load('~/bit-matlab-sim/control_for_estimator.mat')

%psi = angular velocity measured in Fb
%PSI = rotation for time step tk-1 -> tk

%% initializations
%bias offset and bias
bg_true = [0.000001; -0.000005; 0.0000075];
Abg_true = eye(3); %temporary for testing
Cbg_true = eye(3);
CA = Cbg_true*Abg_true;

fs_ekf = 1000;
dt_ekf = 1/fs_ekf;
dt_factor = dt_ekf/dt;

% KVH characteristics
% (https://sites.physics.utoronto.ca/bit/documentation/electronics/data-sheets/dsp-1750-dig-1axis-std-manual.pdf)
kvh_lat = 1.3/1000;
kvh_bias_rate = ((0.05 * pi / 180)/(3600)) * dt_ekf;
kvh_bias_offset = 1*(10 * pi / 180)/3600 * dt_ekf;
kvh_rndwlk = (0.8 * pi /180)/sqrt(1000) * dt_ekf;

dc_b = zeros(3,1) + kvh_bias_offset;
%         omega covariance   alignment drift   %bias drift cov
Q_w = [1 1 1] * 1e10;
Q_bias = [1 1 1] * 1e10;
Qg = diag([Q_w, Q_bias]); %Process (gyro) covariance

Lk = [CA, zeros(3,3); zeros(3,3), eye(3)];
Hk = eye(3,6);
Mk = [eye(3)];
Rk = [eye(3)]*1e15; %measurement covaraince

%current corrected (bias and cal) gyro reading
wb_k = zeros(3,1);

%gyro measurement and gyro bias
bg_hat = zeros(3,1);


zeta_bek = zeros(3,1);
zeta_gb  = zeros(3,1);

psi_hat = zeros(3,1);
psi_hat_mag = 0;
Psi_hat = eye(3);

% num_steps = length(e_hist(1,:))/dt_factor-1;

%% need to process results to only get samples at 1khz
%or figure out indexing.
dt_gyro = 1/1000;
dt_lis = 0.2;
%%
beg = 200;
steps = 100000;

cor_steps = 200;
err_hist = zeros(3,steps);
bias_hist = zeros(3,steps/cor_steps);
bias_err_hist = bias_hist;

% Actual rotation initialization
ck0 = compute_rotation_mat(z_n, y_all(10:18,beg)); 
ck1 = compute_rotation_mat(z_n, y_all(10:18,beg+1));
Psik = eye(3);

%% initialize estimation parameters
ck0_hat = ck0;
ck1_hat = ck0;
P_hat0 = eye(6);
P_hat1 = P_hat0;



for step = beg:beg+steps
    index0 = (step-1) * 1;
    index1 = (step) * 1;

    %% calculate gyro readings 
    ck0  = compute_rotation_mat(z_n, y_all(10:18,index0));
    ck1  = compute_rotation_mat(z_n, y_all(10:18,index1));
    Psik = ck1*ck0';
    [psi_k, psi_mag_k] = rot2axis(Psik);
    Psi_check = axis2rot(psi_k, psi_mag_k);
    if norm(Psi_check-Psik) > 1e-6
        psi_mag_k = -psi_mag_k;
    end


    w_k_true = psi_k * psi_mag_k / dt_gyro;

    %inject bias and offsets
    wg_meas = (CA' * w_k_true) - bg_true;

    %% Propogation
    %gyro reading correction and resulting rotation
    wb_k = CA * (wg_meas + bg_hat);
    psi_hat = wb_k * dt_gyro;
    psi_hat_mag = norm(psi_hat);

    psi_hat = psi_hat/psi_hat_mag;
    cos_pk = cos(psi_hat_mag);
    Psi_hat = (cos_pk*eye(3)) + ((1-cos_pk)*((psi_hat)*(psi_hat'))) ...
            - (sin(psi_hat_mag)*xmat(psi_hat));
    
    % propogate gyro measurement
    ck0_hat = ck1_hat;
    P_hat0 = P_hat1;
    
    ck1_hat = Psi_hat * ck0_hat;
    %check third entry in Fk
    Fk = [Psi_hat,  CA*dt_gyro; ...
          zeros(3,3), eye(3)]; %gyro state jacob
    Lk = [CA*dt_gyro, zeros(3,3); zeros(3,3), eye(3)];%gyro meas jacob

    P_hat1 = (Fk * P_hat0 * Fk') + (Lk * Qg * Lk');
        
    cbe_meas = ck1;
    err = unxmat(eye(3) - (cbe_meas * ck1_hat'));
    err_hist(:,step+1-beg) = err;
%% Correction at 5Hz (every 200 steps)
    if ~(mod(step,cor_steps))

%         cbe_meas = ck1;
%         err = unxmat(eye(3) - (cbe_meas * ck1_hat'));
% %         err_eul = (rotm2eul(cbe_meas * ck1_hat')); %checking err calc?
%         err_hist(:,step+1-beg) = err;
        % calculate kalman gain
        Wk = (Hk*P_hat1* (Hk')) + ( Mk * Rk * (Mk'));
        
        Kk = P_hat1*Hk' / Wk;
        
        [zeta] = Kk * err;
        zeta_bek  = zeta(1:3);
        zeta_bgk = zeta(4:6);
        
        %correct the estimates
        zeta_bek_mag = norm(zeta_bek);
        Zeta_bek = eye(3);
        if zeta_bek_mag 
            zeta_bek = zeta_bek/zeta_bek_mag;
            cos_zk = cos(zeta_bek_mag);
            Zeta_bek = (cos_zk*eye(3)) + ((1-cos_zk)*zeta_bek*zeta_bek') ...
                - (sin(zeta_bek_mag)*xmat(zeta_bek));
        end
        ck1_hat = Zeta_bek*ck1_hat;
 
    
        bg_hat = bg_hat + zeta_bgk;

        bias_hist(:,((step-beg)/cor_steps)+1) = bg_hat;

        bias_err_cur = bg_true - bg_hat

        bias_err_hist(:,((step-beg)/cor_steps)+1) = (bias_err_cur);
        % correct P
        P_hat1 = (P_hat1) - (Kk*Hk*P_hat1) ...
                 - (P_hat1 * Hk' * Kk') + ( Kk * Wk * Kk');
    end

    %calculate error history (need to think about frames)
    
end

% t=dt_ekf:dt_ekf:dt_ekf*num_steps;
% 
% 
figure(1)
tiledlayout(2,3)
nexttile
plot(err_hist(1,:)*206265)
% plot(t,err(1,:),t,err(2,:),t,err(3,:));
legend('roll')
title('Error')
ylabel('RAD')

nexttile
plot(err_hist(2,:)*206265)
% plot(t,err(1,:),t,err(2,:),t,err(3,:));
legend('pitch')
title('Error')
ylabel('RAD')

nexttile
plot(err_hist(3,:)*206265)
% plot(t,err(1,:),t,err(2,:),t,err(3,:));
legend('yaw')
title('Error')
ylabel('RAD')

nexttile
plot(bias_err_hist(1,:))
% plot(t,err(1,:),t,err(2,:),t,err(3,:));
legend('roll bias')
title('Error')
ylabel('RAD')
% 
nexttile
plot(bias_err_hist(2,:))
% plot(t,err(1,:),t,err(2,:),t,err(3,:));
legend('pitch bias')
title('Error')
ylabel('RAD')
% 
nexttile
plot(bias_err_hist(3,:))
% plot(t,err(1,:),t,err(2,:),t,err(3,:));
legend('yaw bias')
title('Error')
ylabel('RAD')
% 

% nexttile
% plot(t,phi_hat(1,:), t,phi(1,:));
% legend('\phi^\^', '\phi')
% title('Roll')
% 
% nexttile
% plot(t,phi_hat(2,:),t,phi(2,:));
% legend('\phi^\^', '\phi')
% title('pitch')
% 
% nexttile
% plot(t,phi_hat(3,:),t,phi(3,:));
% legend('\phi^\^', '\phi')
% title('Yaw')
% 




