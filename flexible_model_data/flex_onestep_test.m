clc
clear

% Run initialization script
load('/home/brad/bit-matlab-sim/flexible_model_data/flex_model');

eta0 = zeros(104,1);
dt = 0.001;
num_steps = 100;


[eta_true] = flex_onestep(eta0, [1, 0.5, 0.5, 0.5, 0.5]', ...
    dt, num_steps, A_f, B_f)