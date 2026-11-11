clear
clc
%%
%[text] ## Load in matrices
load('servo_mats.mat')
% load('/home/bholder/bit-matlab-sim/Optimize/control_matrices.mat')
load('spr_controler_mats.mat')
%%
%[text] ## symbolic to follow
syms tp ty tr dtp dty dtr r_step r_ramp dr_ramp r_osc dr_osc p_step p_ramp dp_ramp p_osc dp_osc y_step y_ramp dy_ramp y_osc dy_osc real
x_roll = [tr; dtr; r_step; r_ramp; dr_ramp; r_osc; dr_osc] %[output:6238dbaf]
x_pit = [tp; dtp; p_step; p_ramp; dp_ramp; p_osc; dp_osc] %[output:9580e1ab]
x_yaw = [ty; dty; y_step; y_ramp; dy_ramp; y_osc; dy_osc] %[output:8c3e7607]
%%
%[text] ## symbolic servo
n = length(Klqr_r);
A_srv = zeros(3*n, 3*n);
A_srv(1:n, 1:n) = A_yaw;
A_srv(n+1:2*n, n+1:2*n) = A_roll;
A_srv((2*n)+1:3*n, (2*n)+1:3*n) = A_pit %[output:0800f0f7]



%Check eigenvalues match
eig(A_srv);
eig(A_roll);
eig(A_pit);
eig(A_yaw);

%assemble B matrices
nr = length(Klqr_r);
nc = 3;
B_srv = zeros(3*nr, 3);
B_srv(1:nr, 1) = B_yaw;
B_srv(nr+1:2*nr, 2) = B_roll;
B_srv((2*nr)+1:3*nr, 3) = B_pit;

K_srv = zeros(3, 3*nr);
K_srv(1, 1:nr) = Klqr_y;
K_srv(2, nr+1:2*nr) = Klqr_r;
K_srv(3, (2*nr)+1:3*nr) = Klqr_p;

A_srv_fb = A_srv - B_srv*K_srv;
eig(A_srv_fb);
x_srv = [x_yaw; x_roll; x_pit];

srv_eqns = A_srv_fb * x_srv;
%%
%[text] ## SPR Mat development

kp = 1.0*diag([Klqr_y(1), Klqr_r(1), Klqr_p(1)]);
kd = 1.0*diag([Klqr_y(2), Klqr_r(2), Klqr_p(2)]);
% kp = diag([3000, 6000, 1500]);
% kd = diag([1950, 2200, 7000]);
% tau_prop = -kp*StS_p*C_pos_comb;
% tau_dam = -kd*StS_p*C_vel_comb;
tau_prop = -kp*C_pos_comb;
tau_dam = -kd*C_vel_comb;
tau_pi = tau_prop+tau_dam;


pid_tdd_posi = B_pos_comb * tau_pi;

% tau_prop = -kp*StS_n*C_pos_comb;
% tau_dam = -kd*StS_n*C_vel_comb;
tau_prop = -kp*C_pos_comb;
tau_dam = -kd*C_vel_comb;

tau_pi = tau_prop+tau_dam;
pid_tdd_neg = B_neg_comb * tau_pi;

A_p = A_full +  pid_tdd_posi;
A_n = A_full + pid_tdd_neg;
% A_p = A_full;
% A_n = A_full;

sys = ss(A_p,B_p,C_vel,0);
[p,r] = isPassive(sys) %[output:7476fa05] %[output:77578701]

sys = ss(A_n,B_n,C_vel,0);
[p,r] = isPassive(sys) %[output:3e11db97] %[output:5e985dab]

% save('/home/bholder/bit-matlab-sim/Optimize/control_matrices.mat', 'A_p', 'B_p', 'C_vel', 'A_n', 'B_n');


%%
addpath('/home/bholder/bit-matlab-sim/Optimize/')
% load('/home/bholder/bit-matlab-sim/Optimize/control_matrices.mat')
% 
% tic
num_in = 5;
num_modes = 6;
num_states = 6+ 2*num_modes;
% 
A_use_p = A_p(1:num_states, 1:num_states);
B_use_p = B_p(1:num_states, :);
C_use = C_vel(:,1:num_states);

%%
%[text] ## spr symbolic

syms tri tyi tpi dtri dtyi dtpi f1 df1 f2 df2 f3 df3 f4 df4 f5 df5 f6 df6 real
xc = sym('xc', [18,1]);

xr = [tyi; tri; tpi; dtyi; dtri; dtpi; f1; df1;f2; df2;f3; df3;f4; df4;f5; df5;f6; df6];
x_spr = [xr;xc];
% 
% spr_eqns = A_polp * x_spr

%%
%[text] ## Build SPR A matrix with lqr built in rather than just P and I
%[text] $x=\\left\\lbrack \\theta \\;\\;\\;\\dot{\\theta \\;} \\;\\;\\;\\eta \\;x\_{\\left\\lbrace \\mathrm{spr},c\\right\\rbrace \\;} \\;x\_{\\mathrm{lqr}} \\;{\\left.\\right\\rbrack }^{T\\;} \\right.${"editStyle":"visual"}
x_tot = [x_spr; y_step; y_ramp; dy_ramp; y_osc; dy_osc;r_step; r_ramp; dr_ramp; r_osc; dr_osc;p_step; p_ramp; dp_ramp; p_osc; dp_osc] %[output:6349e408]
load('joint_dynamics.mat');
A_tot = zeros(51,51);

A_gmb_p = zeros(6,6);
A_gmb_p(4:6,:) = Ap_eval;
A_gmb_p(1:3,4:6) = eye(3);

A_tot(1:18,1:18) = A_full(1:18,1:18);
A_tot(4:6,1:6) = Ap_eval %[output:7315fef8]
A_tot(end-14:end-10, end-14:end-10) = A_yaw(3:7,3:7);
A_tot(end-14:end-10, 1) = A_yaw(3:7,1);
A_tot(end-14:end-10, 4) = A_yaw(3:7,2);

A_tot(end-9:end-5, end-9:end-5) = A_roll(3:7,3:7);
A_tot(end-9:end-5, 2) = A_roll(3:7,1);
A_tot(end-9:end-5, 5) = A_roll(3:7,2);

A_tot(end-4:end, end-4:end) = A_pit(3:7,3:7);
A_tot(end-4:end, 3) = A_pit(3:7,1);
A_tot(end-4:end, 6) = A_pit(3:7,2);
%[text] Calculate tau based on lqr
%[text] $\\tau =-K\_{\\mathrm{lqr}} \\left\\lbrack \\theta \\;\\dot{\\;\\theta \\;} x\_{\\mathrm{lqr}\\;} \\right\\rbrack${"editStyle":"visual"}
B_tot_lqr_pos = zeros(length(x_tot), 3);
B_tot_lqr_pos(1:18,:) = B_pos_comb(1:18,:);
%B_lqr is empty for the servo states

C_vel_tot = zeros(3,51);
C_pos_tot = C_vel_tot;

C_vel_tot(:,1:18) = C_vel_comb(:,1:18);
C_pos_tot(:,1:18) = C_pos_comb(:,1:18);


K_lqr = zeros(3,51);
K_lqr(1,1) = Klqr_y(1);
K_lqr(1,4) = Klqr_y(2);
K_lqr(1,end-14:end-10) = Klqr_y(3:7);
K_lqr(2,2) = Klqr_r(1);
K_lqr(2,5) = Klqr_r(2);
K_lqr(2,end-9:end-5) = Klqr_r(3:7);
K_lqr(3,3) = Klqr_p(1);
K_lqr(3,6) = Klqr_p(2);
K_lqr(3,end-4:end) = Klqr_p(3:7) %[output:57bad1b9]

kp_lqr = K_lqr(:,1:3);
kd_lqr = K_lqr(:,4:6);

K_lqr(:,1:6) = zeros(3,6);

K_lqr = K_lqr + kp_lqr*C_pos_tot + kd_lqr*C_vel_tot %[output:91e5a99d]

A_lqr_tau = -B_tot_lqr_pos * K_lqr;

A_with_lqr = A_tot + A_lqr_tau;

%%
%[text] ## Add in SPR

% A_pol = [A_use, -B_use*Cc; Bc*C_use, Ac];

spr_out = -B_use_p*cc_p;
spr_eqs = [bc_p*C_use, ac_p] %[output:6bffe5f0]

A_spr_lqr_pos = A_with_lqr;
A_spr_lqr_pos(19:36, 1:36) = spr_eqs;
A_spr_lqr_pos(1:18, 19:36) = spr_out;

pos_eigs = eig(A_spr_lqr_pos)

%%
%[text] ## DO FOR NEGATIVE
addpath('/home/bholder/bit-matlab-sim/Optimize/')
% load('/home/bholder/bit-matlab-sim/Optimize/control_matrices.mat')
% 
% tic
num_in = 5;
num_modes = 6;
num_states = 6+ 2*num_modes;
% 
A_use_n = A_n(1:num_states, 1:num_states);
B_use_n = B_n(1:num_states, :);
C_use = C_vel(:,1:num_states);

A_tot = zeros(51,51);

% A_gmb_n = zeros(6,6);
% A_gmb_n(4:6,:) = An_eval;
% A_gmb_n(1:3,4:6) = eye(3);

A_tot(1:18,1:18) = A_full(1:18,1:18);
A_tot(4:6,1:6) = An_eval %[output:27936fa2]
A_tot(end-14:end-10, end-14:end-10) = A_yaw(3:7,3:7);
A_tot(end-14:end-10, 1) = A_yaw(3:7,1);
A_tot(end-14:end-10, 4) = A_yaw(3:7,2);

A_tot(end-9:end-5, end-9:end-5) = A_roll(3:7,3:7);
A_tot(end-9:end-5, 2) = A_roll(3:7,1);
A_tot(end-9:end-5, 5) = A_roll(3:7,2);

A_tot(end-4:end, end-4:end) = A_pit(3:7,3:7);
A_tot(end-4:end, 3) = A_pit(3:7,1);
A_tot(end-4:end, 6) = A_pit(3:7,2);
%[text] Calculate tau based on lqr
%[text] $\\tau =-K\_{\\mathrm{lqr}} \\left\\lbrack \\theta \\;\\dot{\\;\\theta \\;} x\_{\\mathrm{lqr}\\;} \\right\\rbrack${"editStyle":"visual"}
B_tot_lqr_neg = zeros(length(x_tot), 3);
B_tot_lqr_neg(1:18,:) = B_neg_comb(1:18,:);
%B_lqr is empty for the servo states

C_vel_tot = zeros(3,51);
C_pos_tot = C_vel_tot;

C_vel_tot(:,1:18) = C_vel_comb(:,1:18);
C_pos_tot(:,1:18) = C_pos_comb(:,1:18);


K_lqr = zeros(3,51);
K_lqr(1,1) = Klqr_y(1);
K_lqr(1,4) = Klqr_y(2);
K_lqr(1,end-14:end-10) = Klqr_y(3:7);
K_lqr(2,2) = Klqr_r(1);
K_lqr(2,5) = Klqr_r(2);
K_lqr(2,end-9:end-5) = Klqr_r(3:7);
K_lqr(3,3) = Klqr_p(1);
K_lqr(3,6) = Klqr_p(2);
K_lqr(3,end-4:end) = Klqr_p(3:7) %[output:4dc561f4]

kp_lqr = K_lqr(:,1:3);
kd_lqr = K_lqr(:,4:6);

K_lqr(:,1:6) = zeros(3,6);

K_lqr = K_lqr + kp_lqr*C_pos_tot + kd_lqr*C_vel_tot %[output:78fbdfd3]

A_lqr_tau = -B_tot_lqr_neg * K_lqr;

A_with_lqr = A_tot + A_lqr_tau;

%%
%[text] ## Add in SPR

% A_pol = [A_use, -B_use*Cc; Bc*C_use, Ac];

spr_out = -B_use_n*cc_n;
spr_eqs = [bc_n*C_use, ac_n] %[output:85c60d11]

A_spr_lqr_neg = A_with_lqr;
A_spr_lqr_neg(19:36, 1:36) = spr_eqs;
A_spr_lqr_neg(1:18, 19:36) = spr_out;

neg_eigs = eig(A_spr_lqr_neg)





















%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":40.3}
%---
%[output:6238dbaf]
%   data: {"dataType":"symbolic","outputData":{"name":"x_roll","value":"$$ \\left(\\begin{array}{c}\n\\mathrm{tr}\\\\\n\\mathrm{dtr}\\\\\nr_{\\textrm{step}} \\\\\nr_{\\textrm{ramp}} \\\\\n{\\textrm{dr}}_{\\textrm{ramp}} \\\\\nr_{\\textrm{osc}} \\\\\n{\\textrm{dr}}_{\\textrm{osc}} \n\\end{array}\\right) $$"}}
%---
%[output:9580e1ab]
%   data: {"dataType":"symbolic","outputData":{"name":"x_pit","value":"$$ \\left(\\begin{array}{c}\n\\mathrm{tp}\\\\\n\\mathrm{dtp}\\\\\np_{\\textrm{step}} \\\\\np_{\\textrm{ramp}} \\\\\n{\\textrm{dp}}_{\\textrm{ramp}} \\\\\np_{\\textrm{osc}} \\\\\n{\\textrm{dp}}_{\\textrm{osc}} \n\\end{array}\\right) $$"}}
%---
%[output:8c3e7607]
%   data: {"dataType":"symbolic","outputData":{"name":"x_yaw","value":"$$ \\left(\\begin{array}{c}\n\\mathrm{ty}\\\\\n\\mathrm{dty}\\\\\ny_{\\textrm{step}} \\\\\ny_{\\textrm{ramp}} \\\\\n{\\textrm{dy}}_{\\textrm{ramp}} \\\\\ny_{\\textrm{osc}} \\\\\n{\\textrm{dy}}_{\\textrm{osc}} \n\\end{array}\\right) $$"}}
%---
%[output:0800f0f7]
%   data: {"dataType":"matrix","outputData":{"columns":21,"name":"A_srv","rows":21,"type":"double","value":[["0","1.0000","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"]]}}
%---
%[output:7476fa05]
%   data: {"dataType":"textualVariable","outputData":{"header":"logical","name":"p","value":"   1\n"}}
%---
%[output:77578701]
%   data: {"dataType":"textualVariable","outputData":{"name":"r","value":"1.0000"}}
%---
%[output:3e11db97]
%   data: {"dataType":"textualVariable","outputData":{"header":"logical","name":"p","value":"   1\n"}}
%---
%[output:5e985dab]
%   data: {"dataType":"textualVariable","outputData":{"name":"r","value":"1.0000"}}
%---
%[output:6349e408]
%   data: {"dataType":"symbolic","outputData":{"name":"x_tot","value":"$$ \\left(\\begin{array}{c}\n\\mathrm{tyi}\\\\\n\\mathrm{tri}\\\\\n\\mathrm{tpi}\\\\\n\\mathrm{dtyi}\\\\\n\\mathrm{dtri}\\\\\n\\mathrm{dtpi}\\\\\nf_1 \\\\\n{\\textrm{df}}_1 \\\\\nf_2 \\\\\n{\\textrm{df}}_2 \\\\\nf_3 \\\\\n{\\textrm{df}}_3 \\\\\nf_4 \\\\\n{\\textrm{df}}_4 \\\\\nf_5 \\\\\n{\\textrm{df}}_5 \\\\\nf_6 \\\\\n{\\textrm{df}}_6 \\\\\n{\\textrm{xc}}_1 \\\\\n{\\textrm{xc}}_2 \\\\\n{\\textrm{xc}}_3 \\\\\n{\\textrm{xc}}_4 \\\\\n{\\textrm{xc}}_5 \\\\\n{\\textrm{xc}}_6 \\\\\n{\\textrm{xc}}_7 \\\\\n{\\textrm{xc}}_8 \\\\\n{\\textrm{xc}}_9 \\\\\n{\\textrm{xc}}_{10} \\\\\n{\\textrm{xc}}_{11} \\\\\n{\\textrm{xc}}_{12} \\\\\n{\\textrm{xc}}_{13} \\\\\n{\\textrm{xc}}_{14} \\\\\n{\\textrm{xc}}_{15} \\\\\n{\\textrm{xc}}_{16} \\\\\n{\\textrm{xc}}_{17} \\\\\n{\\textrm{xc}}_{18} \\\\\ny_{\\textrm{step}} \\\\\ny_{\\textrm{ramp}} \\\\\n{\\textrm{dy}}_{\\textrm{ramp}} \\\\\ny_{\\textrm{osc}} \\\\\n{\\textrm{dy}}_{\\textrm{osc}} \\\\\nr_{\\textrm{step}} \\\\\nr_{\\textrm{ramp}} \\\\\n{\\textrm{dr}}_{\\textrm{ramp}} \\\\\nr_{\\textrm{osc}} \\\\\n{\\textrm{dr}}_{\\textrm{osc}} \\\\\np_{\\textrm{step}} \\\\\np_{\\textrm{ramp}} \\\\\n{\\textrm{dp}}_{\\textrm{ramp}} \\\\\np_{\\textrm{osc}} \\\\\n{\\textrm{dp}}_{\\textrm{osc}} \n\\end{array}\\right) $$"}}
%---
%[output:7315fef8]
%   data: {"dataType":"matrix","outputData":{"columns":51,"exponent":"4","name":"A_tot","rows":51,"type":"double","value":[["0","0","0","0.0001","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"]]}}
%---
%[output:57bad1b9]
%   data: {"dataType":"matrix","outputData":{"columns":51,"exponent":"3","name":"K_lqr","rows":3,"type":"double","value":[["1.4159","0","0","1.8886","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"]]}}
%---
%[output:91e5a99d]
%   data: {"dataType":"matrix","outputData":{"columns":51,"exponent":"3","name":"K_lqr","rows":3,"type":"double","value":[["1.4159","0","0","1.8886","0","0","0.0118","0.0157","0.0093","0.0124","-0.0064","-0.0086","0.0000","0.0000","0.0000","0.0001","-0.0035","-0.0047","0","0","0","0","0","0","0","0","0","0","0","0"]]}}
%---
%[output:6bffe5f0]
%   data: {"dataType":"matrix","outputData":{"columns":36,"exponent":"4","name":"spr_eqs","rows":18,"type":"double","value":[["0","0","0","0.0000","-0.0000","-0.0000","0","0.0000","0","0.0000","0","0.0000","0","-0.0000","0","-0.0000","0","0.0000","0","0","0","0.0001","0","0","0","0","0","0","0","0"]]}}
%---
%[output:27936fa2]
%   data: {"dataType":"matrix","outputData":{"columns":51,"exponent":"4","name":"A_tot","rows":51,"type":"double","value":[["0","0","0","0.0001","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"]]}}
%---
%[output:4dc561f4]
%   data: {"dataType":"matrix","outputData":{"columns":51,"exponent":"3","name":"K_lqr","rows":3,"type":"double","value":[["1.4159","0","0","1.8886","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"]]}}
%---
%[output:78fbdfd3]
%   data: {"dataType":"matrix","outputData":{"columns":51,"exponent":"3","name":"K_lqr","rows":3,"type":"double","value":[["1.4159","0","0","1.8886","0","0","0.0118","0.0157","0.0093","0.0124","-0.0064","-0.0086","0.0000","0.0000","0.0000","0.0001","-0.0035","-0.0047","0","0","0","0","0","0","0","0","0","0","0","0"]]}}
%---
%[output:85c60d11]
%   data: {"dataType":"matrix","outputData":{"columns":36,"exponent":"4","name":"spr_eqs","rows":18,"type":"double","value":[["0","0","0","0.0000","-0.0000","-0.0000","0","0.0000","0","0.0000","0","0.0000","0","-0.0000","0","-0.0000","0","0.0000","0","0","0","0.0001","0","0","0","0","0","0","0","0"]]}}
%---
