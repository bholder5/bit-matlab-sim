cd('/home/bholder/bit-matlab-sim')
addpath('/home/bholder/bit-matlab-sim/Miscellaneous')
addpath('/home/bholder/bit-matlab-sim/flexible_model_data/')
addpath('/home/bholder/bit-matlab-sim/Plant_functions/')
load_sl_glibc_patch


%%
%[text] ## lqr first
%[text] # Servomechanism control problem
%[text] ## building disturbance signal -1 input
%[text] \\omega = Cbar z1
%[text] dot z1 = Abar1 z1
freq_osc1 = 0.0* 2*pi;
freq_osc3 = 0.088 * 2*pi;
freq_osc2 = 0.033* 2*pi;
freq_osc4 = 0.9 * 2*pi;

ad1 = [0 1;
       -freq_osc1^2 0];
ad2 = [0 1;
       -freq_osc2^2 0];

ad3 = [0 1;
       -freq_osc3^2 0];
ad4 = [0 1;
       -freq_osc4^2 0];

cdist1 = [0 1] %[output:7d9cd90e]

nd = 2;
nr = 0;

ns = 2*nd + nr;
ad_comb = zeros(ns,ns) %[output:1d8bc25c]
ad_comb(nr+1:nr+2,nr+1:nr+2) = ad1;
ad_comb(nr+3:nr+4,nr+3:nr+4) = ad2;
% ad_comb(nr+5:nr+6,nr+5:nr+6) = ad3;
% ad_comb(nr+7:nr+8,nr+7:nr+8) = ad4;

eig(ad_comb)

%%
%[text] ## canonical form
A1 = sym(ad_comb);

syms x

poly1 = charpoly(A1,x);



p = zeros(ns,ns);
p(1:(ns-1),2:ns) = eye(ns-1);

tmp = -double(coeffs(poly1, 'All')) %[output:8f3a6666]
p(ns, :) = tmp(end:-1:2) %[output:67fd4448]
eig(p)

%%
%[text] ## Choose R and confirm controlability
rs_v = zeros(ns,1);
if nr
    rs_v(1) = 0
end

for loc = 1:nd
    rs_v(nr+(((loc)*2))) = 1;
end
rs_p = zeros(ns,1) %[output:8d8b2288]

rs_p = [ 1 0 1 0]';% rs_p(1+nr:(ns-1)) = rs_v(2+nr:ns)% This is only valid until ref signal is added in.
% rs_p = [1 0]' 1 0
% rs_v(1) = 1;
rs = [rs_p,rs_v] %[output:3c4b5acc]
rank(ctrb(p, rs_v)) %[output:8a2a5ab6]
%%
%[text] ## Assemble for 3 inputs
sstr = zeros(ns*3,ns*3);
sstr(1:ns,1:ns) = p;
sstr(ns+1:2*ns,ns+1:2*ns) = p;
sstr(((2*ns)+1):(3*ns),(2*ns+1):3*ns) = p;

rstr = [rs zeros(ns,4);zeros(ns,2), rs, zeros(ns,2); zeros(ns,4), rs];

%%
%[text] ## Load in A and B for ss system
load("ss_mats.mat")

%%
%[text] ## full control design
Aneut %[output:786c47fb]
eig(Aneut)
Bneut * [1 0 0]' * [1 0 0 0 0 0] *10;
Akp = Aneut + Bneut * [1 0 0]' * [1 0 0 0 0 0] *10;
Acm = [Aneut zeros(6,3*ns);
        rstr*Cneut, sstr];
Bcm = [Bneut; zeros(3*ns,3)];

rank(ctrb(Acm,Bcm)) %not controllable but stabilizable? %[output:8c3c6738]

Ccm = [Cneut, zeros(6,3*ns);
        zeros(3*ns,6), eye(3*ns)];
rank(obsv(Acm, Ccm)) %not observable (missing 1 state) likely related to lake of spring/position on yaw %[output:1712377f]


% pls = -0.1*[1:1:27];
% apoles = [0, 0, 0 + 0.719422554386278i, 0 - 0.719422554386278i, 0 + 0.515116797080478i, 0 - 0.515116797080478i];

% plc_pls = [apoles, pls];
% K = place(Acm, Bcm, plc_pls)
% Kcm = place(Acm, Bcm, pls)

% Lcm = place(Acm', Ccm', 10*pls);
% Lcm = Lcm';

%%
%[text] ## derie matrices for simulink controller
% Aob = Acm - Lcm*Ccm;
% % Bob = Bcm*Kcm;
% Cob = eye(30);
% 
% % Kcm;
% 
% Qcm = Kcm(:,7:end);

rout = rstr' %[output:1f96afc8]
ns_cm = size(Acm);
ns_cm = ns_cm(1);
% 
% qlqr = eye(ns_cm)/1000;
% qlqr(1,1) = .000001;
% qlqr(2,2) = .000001;
% qlqr(3,3) = .000001;
% qlqr(4,4) = .000001;
% qlqr(5,5) = .000001;
% qlqr(6,6) = .000001;

% ROLL 

% 
% qlqr = 1*qlqr/1;
% rlqr = 1000000*eye(3);

% Klqr = lqr(Acm, Bcm, qlqr, rlqr)
%%
%[text] ## speed prifle derivatives
roll_slope = ((0.01*erf(0.25*15*0.01))-(0.01*erf(0.25*15*-0.01)))/0.02 %[output:494942d5]
pitch_slope = ((0.05*erf(0.025*80*0.01))-(0.05*erf(0.025*80*-0.01)))/0.02 %[output:073198ca]
yaw_slope = ((0.01*erf(6*0.01))-(0.01*erf(6*-0.01)))/0.02 %[output:2bea07b4]
%%
%[text] ## new try to do each axis separate
r_roll = [rs_v,rs_p];

B_roll = [0; Bneut(5,2); zeros(ns,1)];

A_roll = zeros(2+ns, 2+ns);
A_roll(1,2) = 1;
A_roll(2,1) = Aneut(5,2);
A_roll(3:end, 3:end) = p;
C_roll = zeros(2,8);
C_roll = eye(2);

A_roll(3:end, 1:2) = r_roll*C_roll %[output:396cf774]
rank(ctrb(A_roll, B_roll)) %[output:2d2983e1]
% syms r rd rf z1 zd1 z2 zd2 z3 zd3 'real'
% 
% state_roll = [r;rd; z1;zd1;];
% A_roll*state_roll;
ns_cm = size(A_roll);
ns_cm = ns_cm(1);

qlqr = eye(ns_cm);
qlqr(1,1) = 100.001;
qlqr(2,2) = 100.001;
qlqr(3,3) = 150000;
qlqr(4,4) = 100000;
qlqr(5,5) = .01;
qlqr(6,6) = .01;
% qlqr(8,8) = 0.00000001;
% qlqr(9,9) = 0.00000001;
% ROLL 


qlqr = 1*qlqr/1;
rlqr = 1000;

Klqr_r = lqr(A_roll, B_roll, qlqr, rlqr) %[output:4fb151b1]

A_roll2 = A_roll - B_roll*Klqr_r;
eig(A_roll2);
eig(A_roll);

%%
%[text] ## Pitch
r_pit = [rs_v,rs_p];

B_pit = [0; Bneut(6,3); zeros(ns,1)];

A_pit = zeros(2+ns, 2+ns);
A_pit(1,2) = 1;
A_pit(2,1) = Aneut(6,3);
A_pit(3:end, 3:end) = p;

C_pit = eye(2);

A_pit(3:end, 1:2) = r_pit*C_pit %[output:06933b17]
rank(ctrb(A_pit, B_pit));
% syms tp tpd rf z1 zd1 z2 zd2 z3 zd3 'real'
% 
% state_pit = [tp;tpd; z1;zd1;];
% A_pit*state_pit;
ns_cm = size(A_pit);
ns_cm = ns_cm(1);

qlqr = eye(ns_cm);
qlqr(1,1) = 1000.0;
qlqr(2,2) = 1000.0;
% qlqr(3,3) = 800;
qlqr(3,3) = 10000;
qlqr(4,4) = 1;
qlqr(5,5) = .0001;
qlqr(6,6) = .0001;
% qlqr(8,8) = 0.00000001;
% qlqr(9,9) = 0.00000001;
% ROLL 


qlqr = 1*qlqr/1;
rlqr = 300;

Klqr_p = lqr(A_pit, B_pit, qlqr, rlqr) %[output:75b019ba]

A_pit2 = A_pit - B_pit*Klqr_p;
eig(A_pit2);
eig(A_pit);

%%
%[text] ## Yaw
r_yaw = [rs_v,rs_p];

B_yaw = [0; -Bneut(4,1); zeros(ns,1)];

A_yaw = zeros(2+ns, 2+ns);
A_yaw(1,2) = 1;
% A_yaw(2,1) = 200*Bneut(4,1);
A_yaw(3:end, 3:end) = p %[output:4502692b]

C_yaw = eye(2);

A_yaw(3:end, 1:2) = r_yaw*C_yaw %[output:38de2d11]
rank(ctrb(A_yaw, B_yaw));
% syms tp tpd rf z1 zd1 z2 zd2 z3 zd3 'real'
% 
% state_yaw = [tp;tpd; z1;zd1;];
% A_yaw*state_yaw;
ns_cm = size(A_yaw);
ns_cm = ns_cm(1);

qlqr = eye(ns_cm);
qlqr(1,1) = 100.1;
qlqr(2,2) = 100.1;
% qlqr(3,3) = 30000.5;
qlqr(3,3) = 100000;
qlqr(4,4) = 1;
qlqr(5,5) = .1;
qlqr(6,6) = .1;
% qlqr(8,8) = 0.00000001;
% qlqr(9,9) = 0.00000001;
% ROLL 


qlqr = 1*qlqr/1;
rlqr = 500000;

Klqr_y = lqr(A_yaw, B_yaw, qlqr, rlqr) %[output:8034714d]

A_yaw2 = A_yaw - B_yaw*Klqr_y;
eig(A_yaw2);
eig(A_yaw);
%%
% save('servo_mats.mat', "A_yaw", "B_yaw", "Klqr_y", "A_pit", "B_pit", "Klqr_p", "A_roll", "B_roll", "Klqr_r");
%%
%[text] ## Generate Rust Files
% generate_rust_code_to_file('/home/bholder/gigabit_code/adcs/src/control/servo/servo.rs', p, rs, Klqr_y, Klqr_r, Klqr_p);

%[text] START SPR
%%

syms t1 t2 t3 t4 t5 t6 t7 t8 t9 real
syms dt1 dt2 dt3 dt4 dt5 dt6 dt7 dt8 dt9 real

zer = sym(0) %[output:7c291a6a]

thet = [t1 t2 t3 t4 t5 t6 t7 t8 t9]' %[output:8584dbbd]
d_thet = [dt1; dt2; dt3; dt4; dt5; dt6; dt7; dt8; dt9];

z_n = sym([0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0; %[output:group:01e0952b] %[output:39da53ae]
       0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0; %[output:39da53ae]
       1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]) %[output:group:01e0952b] %[output:39da53ae]

s7 = zeros(3,9);
for i = 1:9
    Cn = axis2rot_symb(z_n(:,i), thet(i));
    s7(:,i) = z_n(:,i);
    s7 = Cn*s7;    
end

%%
s = zeros(3,9);
for i = 1:7
    Cn = axis2rot_symb(z_n(:,i), thet(i));
    s(:,i) = z_n(:,i);
    s = Cn*s;    
end

tau_y = sym('tau_y') %[output:3977a7cd]
syms hs1 hs2 hs dhs1 dhs2 dhs real

h_s = [zer; zer; hs];
d_hs = [zer; zer; -tau_y];

% z7 = [0.01;0.015;1];
% z7 = z7/sqrt(z7'*z7)
z7 = z_n(:,7);


% d_hs = tau_rw*z7;

r = (s')*d_hs;

R = -(s') * xmat(h_s) * s;
R = simplify(R);
%%
[ndof, g0, r_n1_n, z_n, p_n, m_n, c_n, ...
    i_n, m_w_n,  i_rw, bear_k_cst, bear_c_cst, k_d, b_d, ...
    w_rw_max, w_rw_nom, hs_rw, hs_rw_max, a_flex, b_flex, a_df, b_df] = init_func();

M = compute_mass_matrix_symb(thet, z_n, r_n1_n, m_w_n, p_n);

% Mb = zeros(6,6);
% Mb = M([1:3,7:9], [1:3, 7:9]);
% Mb = subs(Mb, t1, zer);
% Mb = subs(Mb, t6, zer);
% Mb = subs(Mb, t4, zer);
% Mb = subs(Mb, t5, zer);
% Mb_i = inv(Mb);

M = subs(M, t1, zer);
M = subs(M, t2, zer);
M = subs(M, t3, zer);
% 
% M6 = M(4:9, 4:9);
% M6i = inv(M6);

M = subs(M, t4, zer);
M = subs(M, t5, zer);
M = subs(M, t6, zer);
M = subs(M, t7, zer);

Mi = inv(M);

Pot = compute_potential_energy_term_symb(thet, c_n, z_n, m_n, r_n1_n, g0);

% 
spring = k_d.*thet;

syms tau_p tau_r tau_piv

tau = sym(zeros(9,1));
% tau(6) = tau_piv;
tau(8) = tau_r;
tau(9) = tau_p;

%%

md = Mi*(tau-(R*d_thet)-r-spring-Pot);
mod = md(7:9);
% mod = md;
% mod = md(4:9);
% mod(1) = md(2) + md(4);12
% mod(2) = md(3) + md(5);
% 
% A_pend = [diff(mod, t6), diff(mod, t7), ...
%     diff(mod, t8), diff(mod, t9),...
%     diff(mod, dt6),   ...
%     diff(mod, dt7),  diff(mod, dt8), diff(mod, dt9),diff(mod, hs)];
% 
A_pend = [diff(mod, t7), ...
    diff(mod, t8), diff(mod, t9),...
    diff(mod, dt7),  diff(mod, dt8), diff(mod, dt9)];

% mod2 = [mod(6); mod(7); mod(8); mod(9)]
% mod = mod2;
% A_pend2 = [diff(mod, t6), diff(mod, t7), ...
%     diff(mod, t8), diff(mod, t9), diff(mod, dt6),   ...
%     diff(mod, dt7),  diff(mod, dt8), diff(mod, dt9),diff(mod, hs)];


% A_gond = A_pend(7:9, 13:19);
A_gond = A_pend;

A_gond = subs(A_gond, t1, zer);
A_gond = subs(A_gond, t2, zer);
A_gond = subs(A_gond, t3, zer);
A_gond = subs(A_gond, t4, zer);
A_gond = subs(A_gond, t5, zer);
A_gond = subs(A_gond, t6, zer);
A_gond = subs(A_gond, t7, zer);
% A_gond = subs(A_gond, t8, 0.1);
% A_gond = subs(A_gond, t9, -0.4*180/pi);
A_gond = subs(A_gond, dt1, zer);
A_gond = subs(A_gond, dt2, zer);
A_gond = subs(A_gond, dt3, zer);
A_gond = subs(A_gond, dt4, zer);
A_gond = subs(A_gond, dt5, zer);
A_gond = subs(A_gond, dt6, zer);
A_gond = subs(A_gond, dt7, zer);
A_gond = subs(A_gond, dt8, zer);
A_gond = subs(A_gond, dt9, zer);

% A_gond = subs(A_gond, hs, 2*pi*4.5);
% A_gond = subs(A_gond, hs2, zer);
% A_gond = subs(A_gond, dhs1, zer);
% A_gond = subs(A_gond, dhs2, zer);
% A_gond = vpa(simplify(A_gond),2)

B_pend = [diff(mod, tau_y), diff(mod, tau_r), diff(mod, tau_p)];

B_gond = B_pend;

B_gond = subs(B_gond, t1, zer);
B_gond = subs(B_gond, t2, zer);
B_gond = subs(B_gond, t3, zer);
B_gond = subs(B_gond, t4, zer);
B_gond = subs(B_gond, t5, zer);
% B_gond = subs(B_gond, t6, zer);
% B_gond = subs(B_gond, t7, zer);
% B_gond = subs(B_gond, t8, zer);
% B_gond = subs(B_gond, t9, zer);
B_gond = subs(B_gond, dt1, zer);
B_gond = subs(B_gond, dt2, zer);
B_gond = subs(B_gond, dt3, zer);
B_gond = subs(B_gond, dt4, zer);
B_gond = subs(B_gond, dt5, zer);
B_gond = subs(B_gond, dt6, zer);
B_gond = subs(B_gond, dt7, zer);
B_gond = subs(B_gond, dt8, zer);
B_gond = subs(B_gond, dt9, zer);

% B_gond = subs(B_gond, hs, 2*pi*4.5);
B_gond = subs(B_gond, hs2, zer);
B_gond = subs(B_gond, dhs1, zer);
B_gond = subs(B_gond, dhs2, zer);
% B_gond = vpa(simplify(B_gond),2)

roll_torque = 0.1*k_d(8);

Ap = subs(A_gond, t7, zer);
Ap = subs(Ap, t8, 0.1);
Ap = subs(Ap, t9, -40*pi/180);
Ap = subs(Ap, tau_p, zer);
Ap = subs(Ap, tau_y, zer);
Ap = subs(Ap, tau_piv, zer);
Ap = subs(Ap, tau_r, roll_torque);
Ap = subs(Ap, hs, 2*pi*4.5);


An = subs(A_gond, t7, 0);
An = subs(An, t8, 0.1);
An = subs(An, t9, -0.7);
An = subs(An, tau_p, zer);
An = subs(An, tau_y, zer);
An = subs(An, tau_r, -roll_torque);
An = subs(An, hs, 2*pi*4.5);

A_neg1 = vpa(An, 4);
A_pos1 = vpa(Ap, 4);

% vpa((An + Ap)/2, 4);

Bp = subs(B_gond, t7, zer);
Bp = subs(Bp, t8, 0.1);
Bp = subs(Bp, t9, -40*pi/180);
% Bp = subs(Bp, tau_p, zer);
% Bp = subs(Bp, tau_rw, zer);
% Bp = subs(Bp, tau_r, roll_torque);

B_pos1 = vpa(Bp, 4) %[output:2a4a71d1]

Bn = subs(B_gond, t7, zer);
Bn = subs(Bn, t8, -0.1);
Bn = subs(Bn, t9, -40*pi/180);
% Bp = subs(Bp, tau_p, zer);
% Bp = subs(Bp, tau_rw, zer);
% Bp = subs(Bp, tau_r, roll_torque);

B_neg1 = vpa(Bn, 4) %[output:25404894]


%%
%[text] ## build gamma and nu column vectors. gamm is velocity and nu is position
aflex = a_mf_func;
bflex = b_mf_func;
l = length(bflex(:,1));

gam_y = [0 0 0 1 0 0 bflex(:,1)']';
nu_y = [1 0 0 0 0 0 bflex(2:l,1)', 0]';

gam_bw = [0 0 0 0 1 0 bflex(:,2)']';
nu_bw = [0 1 0 0 0 0 bflex(2:l,2)', 0]';

gam_st = [0 0 0 0 1 0 bflex(:,3)']';
nu_st = [0 1 0 0 0 0 bflex(2:l,3)', 0]';

gam_pt = [0 0 0 0 0 1 bflex(:,4)']';
nu_pt = [0 0 1 0 0 0 bflex(2:l,4)', 0]';

gam_sb = [0 0 0 0 0 1 bflex(:,5)']';
nu_sb = [0 0 1 0 0 0 bflex(2:l,5)', 0]';

%%
%[text] ## Build C and B matrix for velocity and position
C_vel = [gam_y'; %[output:group:1750611b] %[output:8ec3150f]
         gam_bw'; %[output:8ec3150f]
         gam_st'; %[output:8ec3150f]
         gam_pt'; %[output:8ec3150f]
         gam_sb'] %[output:group:1750611b] %[output:8ec3150f]

C_pos = [nu_y'; %[output:group:36a67867] %[output:0e8c30cc]
         nu_bw'; %[output:0e8c30cc]
         nu_st'; %[output:0e8c30cc]
         nu_pt'; %[output:0e8c30cc]
         nu_sb'] %[output:group:36a67867] %[output:0e8c30cc]

B_p = C_vel' %[output:4f5f2c87]
B_p(4:6,:) = [B_pos1(:,1),B_pos1(:,2),B_pos1(:,2),B_pos1(:,3),B_pos1(:,3)]  %[output:60e6b165]

B_n = C_vel' %[output:61341b1e]
B_n(4:6,:) = [B_neg1(:,1),B_neg1(:,2),B_neg1(:,2),B_neg1(:,3),B_neg1(:,3)]  %[output:7993f976]

%%
%[text] ## Now build C and B matrices for calculating effect of PI and flex bearings
C_vel_comb = [gam_y'; %[output:group:118b89f0] %[output:393a8e5a]
              mean([gam_bw,gam_st],2)'; %[output:393a8e5a]
              mean([gam_pt,gam_sb],2)'] %[output:group:118b89f0] %[output:393a8e5a]

C_pos_comb = [nu_y'; %[output:group:3c2979db] %[output:43103c0d]
              mean([nu_bw,nu_st],2)'; %[output:43103c0d]
              mean([nu_pt,nu_sb],2)'] %[output:group:3c2979db] %[output:43103c0d]

B_pos_comb = C_vel_comb' %[output:1ef4e237]
B_pos_comb(1:6,:) =[zeros(3,3); eval(Bp)] %[output:8d802426]

B_neg_comb = C_vel_comb' %[output:106903b1]
B_neg_comb(1:6,:) =[zeros(3,3); eval(Bn)] %[output:37bc266b]

%%
%[text] ## build StS matrices
[S9, S8, S7] = compute_s_mat_symb(thet, z_n);
S_c = sym(zeros(3,9));

S_c(1,:) = S7(3,:);
S_c(2,:) = S8(1,:);
S_c(3,:) = S9(2,:);

S_c = subs(S_c, t1, 0);
S_c = subs(S_c, t2, 0);
S_c = subs(S_c, t3, 0);
S_c = subs(S_c, t4, 0);
S_c = subs(S_c, t5, 0);
S_c = subs(S_c, t6, 0);
S_c = subs(S_c, t7, 0);
S_cp = subs(S_c, t9, -40*pi/180);

ang_vel_p = S_cp*d_thet;
S = S_cp(:,7:9);
S = subs(S,t8, 0.1) %[output:3aa1b2b9]
S = eval(S);

StS_p = S'*S;
% 
% stiffness = m79*-kp*StS_p;
% damping = m79*-kd*StS_p;

Sn = S_cp(:,7:9);
Sn = subs(Sn,t8, -0.1) %[output:1b2ff5cc]
Sn = eval(Sn);
StS_n = Sn'*Sn;
%%
%[text] ## Build A matrix with rigid coordinates (not joint angles)
A_full = zeros(110,110);
A_full(1:3,4:6) = eye(3);
A_full(7:110,7:110) = aflex;
% save('/home/bholder/bit-matlab-sim/Optimize/control_matrices.mat', 'A_full', 'B_pos_comb', 'B_neg_comb', 'C_pos_comb', 'C_vel_comb', 'B_p', 'B_n', 'C_vel');
% load('/home/bholder/bit-matlab-sim/Optimize/control_matrices.mat');
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
% tau_dam = -kd*StS_n*C_vel_comb;DFADSFASD
tau_prop = -kp*C_pos_comb;
tau_dam = -kd*C_vel_comb;

tau_pi = tau_prop+tau_dam;
pid_tdd_neg = B_neg_comb * tau_pi;

A_p = A_full +  pid_tdd_posi;
A_n = A_full + pid_tdd_neg;
% A_p = A_full;
% A_n = A_full;

sys = ss(A_p,B_p,C_vel,0);
[p,r] = isPassive(sys) %[output:58a347c1] %[output:2520351e]

sys = ss(A_n,B_n,C_vel,0);
[p,r] = isPassive(sys) %[output:5e4d3f57] %[output:5367a71e]

% save('/home/bholder/bit-matlab-sim/Optimize/control_matrices.mat', 'A_p', 'B_p', 'C_vel', 'A_n', 'B_n');


%%
addpath('/home/bholder/bit-matlab-sim/Optimize/')
% load('/home/bholder/bit-matlab-sim/Optimize/control_matrices.mat')
% 
tic
num_in = 5;
num_modes = 6;
num_states = 6+ 2*num_modes;

A_use = A_p(1:num_states, 1:num_states) %[output:0ca3bfce]
B_use = B_p(1:num_states, :) %[output:551df8e1]
C_use = C_vel(:,1:num_states) %[output:2fe15d6e]
% % 
% qr = diag([450, 800, 1000,...
%            1500,7500, 1000,...
%            10, 100,...
%            10, 100,...
%            % 1, 10,...
%            % 1, 10 ...
%            ]);
% 
% ql = diag([0.005, .4, .1,...
%            0.01, 0.001, 0.1,...
%            .1, .1,...
%            .1, .1, ...
%            % 1, 1, ...
%            % 1, 1 ...
%            ]);

qr = diag( [.01, .01, .01,...
           .01,.01, .01,...
           100, 1000,...
           500, 4000,...
           1, 10,...
           1, 10,...
           1, 10,...
           1, 10,...
           % 1, 10,...
           % 1, 10 ...
           ]);

ql = diag([1, 1, 1,...
           1, 1, 1,...
           .1, .1,...
           .05, .05, ...
           .05, .05,...
           .01, .01,...
           .01, .01,...
           .005, .005,...
           % 1, 1, ...
           % 1, 1 ...
           ]);

% ql = inv(qr)*10;
r_lqr = 1000.00* diag([1, 1, 1, 1, 1]);
[K,P1,Pol] = lqr(A_use,B_use,qr,r_lqr,0);
Cc = K;
%%\ make closed loop A
Ac = A_use - (B_use * Cc);
P2 = lyap(Ac',ql);
Bc = inv(P2)*Cc';

cc_p = Cc;
bc_p = Bc;
ac_p = Ac;
A_pol = [A_use, -B_use*Cc; Bc*C_use, Ac];
eigc = eig(Ac);
eigf = eig(A_pol);
sort(eigc, 'ComparisonMethod', 'real')
sort(eigf, 'ComparisonMethod', 'real')
max(real(eigf)) %[output:2aabd6bc]
%%
%[text] ## Negative


% generate_rust_code_to_file('/home/bholder/gigabit_code/adcs/src/control/lqr/pos.rs', A_use, B_use, ac_p, bc_p, cc_p);

% [xout2, eigc, eigf] = opt_ql(a_ss_n, b_ss_n, c_ss_n, qr, r)

% ql = diag(xout2(1:13));
% r = diag(xout2(14:18));


A_use = A_n(1:num_states, 1:num_states);
B_use = B_n(1:num_states, :);
C_use = C_vel(:,1:num_states);

[K,P1,Pol] = lqr(A_use,B_use,qr,r_lqr,0);

Cc = K;

%%\ make closed loop A

Ac = A_use - (B_use * Cc);
eig(Ac)

P2 = lyap(Ac',ql);

Bc = inv(P2)*Cc';

cc_n = Cc;
bc_n = Bc;
ac_n = Ac;
% 
% A_use = a_ss_n;
% % A_use(5,5) = -0.001;
% B_use = b_ss_n;
% C_use = c_ss_n;

A_pol = [A_use, -B_use*Cc; Bc*C_use, Ac];

eigc = eig(Ac);
eigf = eig(A_pol);
max(real(eigf)) %[output:267b182c]


sort(eigc, 'ComparisonMethod', 'real')
sort(eigf, 'ComparisonMethod', 'real')

% generate_rust_code_to_file('/home/bholder/gigabit_code/adcs/src/control/lqr/neg.rs', A_use, B_use, ac_n, bc_n, cc_n);


toc %[output:7fb21ebc]
% 
% tic
% generateControlFiles(450, 800, 1000, 1500, 7500, 1000, 0.005, 0.4, 0.1, 0.01, 0.001, 0.1);
% toc
% save('spr_controler_mats.mat', 'ac_p', 'bc_p', 'cc_p', 'ac_n', 'bc_n', 'cc_n', 'B_n', 'B_p', 'C_vel', 'C_vel_comb', 'C_pos_comb', 'B_pos_comb', 'A_full', 'B_neg_comb');

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline","rightPanelPercent":30.9}
%---
%[output:7d9cd90e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"cdist1","rows":1,"type":"double","value":[["0","1"]]}}
%---
%[output:1d8bc25c]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"ad_comb","rows":4,"type":"double","value":[["0","0","0","0"]]}}
%---
%[output:8f3a6666]
%   data: {"dataType":"matrix","outputData":{"columns":5,"name":"tmp","rows":1,"type":"double","value":[["-1.0000","0","-0.0430","0","0"]]}}
%---
%[output:67fd4448]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"p","rows":4,"type":"double","value":[["0","1.0000","0","0"]]}}
%---
%[output:8d8b2288]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"rs_p","rows":4,"type":"double","value":[["0"],["0"],["0"],["0"]]}}
%---
%[output:3c4b5acc]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"rs","rows":4,"type":"double","value":[["1","0"]]}}
%---
%[output:8a2a5ab6]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"4"}}
%---
%[output:786c47fb]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"Aneut","rows":6,"type":"double","value":[["0","0","0","1.0000","0","0"]]}}
%---
%[output:8c3c6738]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"16"}}
%---
%[output:1712377f]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"18"}}
%---
%[output:1f96afc8]
%   data: {"dataType":"matrix","outputData":{"columns":12,"name":"rout","rows":6,"type":"double","value":[["1","0","1","0","0","0","0","0","0","0","0","0"]]}}
%---
%[output:494942d5]
%   data: {"dataType":"textualVariable","outputData":{"name":"roll_slope","value":"0.0423"}}
%---
%[output:073198ca]
%   data: {"dataType":"textualVariable","outputData":{"name":"pitch_slope","value":"0.1128"}}
%---
%[output:2bea07b4]
%   data: {"dataType":"textualVariable","outputData":{"name":"yaw_slope","value":"0.0676"}}
%---
%[output:396cf774]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"A_roll","rows":6,"type":"double","value":[["0","1.0000","0","0","0","0"]]}}
%---
%[output:2d2983e1]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"6"}}
%---
%[output:4fb151b1]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"Klqr_r","rows":1,"type":"double","value":[["261.4215","449.4079","12.2474","63.4670","153.3199","235.6028"]]}}
%---
%[output:06933b17]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"A_pit","rows":6,"type":"double","value":[["0","1.0000","0","0","0","0"]]}}
%---
%[output:75b019ba]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"Klqr_p","rows":1,"type":"double","value":[["62.4775","178.9527","5.7735","27.7351","64.1040","92.9484"]]}}
%---
%[output:4502692b]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"A_yaw","rows":6,"type":"double","value":[["0","1.0000","0","0","0","0"]]}}
%---
%[output:38de2d11]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"A_yaw","rows":6,"type":"double","value":[["0","1.0000","0","0","0","0"]]}}
%---
%[output:8034714d]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"Klqr_y","rows":1,"type":"double","value":[["431.4786","939.6873","0.4472","5.7918","20.5671","124.0620"]]}}
%---
%[output:7c291a6a]
%   data: {"dataType":"symbolic","outputData":{"name":"zer","value":"$$0$$"}}
%---
%[output:8584dbbd]
%   data: {"dataType":"symbolic","outputData":{"name":"thet","value":"$$ \\left(\\begin{array}{c}\nt_1 \\\\\nt_2 \\\\\nt_3 \\\\\nt_4 \\\\\nt_5 \\\\\nt_6 \\\\\nt_7 \\\\\nt_8 \\\\\nt_9 \n\\end{array}\\right) $$"}}
%---
%[output:39da53ae]
%   data: {"dataType":"symbolic","outputData":{"name":"z_n","value":"$$ \\left(\\begin{array}{ccccccccc}\n0 & 1 & 0 & 1 & 0 & 0 & 0 & 1 & 0\\\\\n0 & 0 & 1 & 0 & 1 & 0 & 0 & 0 & 1\\\\\n1 & 0 & 0 & 0 & 0 & 1 & 1 & 0 & 0\n\\end{array}\\right) $$"}}
%---
%[output:3977a7cd]
%   data: {"dataType":"symbolic","outputData":{"name":"tau_y","value":"$$ \\tau_y $$"}}
%---
%[output:2a4a71d1]
%   data: {"dataType":"symbolic","outputData":{"name":"B_pos1","value":"$$ \\left(\\begin{array}{ccc}\n0.001025 & \\text{4.126e-5} & -0.0001156\\\\\n\\text{4.126e-5} & 0.004228 & -\\text{1.44e-6}\\\\\n-0.0001156 & -\\text{1.44e-6} & 0.008267\n\\end{array}\\right) $$"}}
%---
%[output:25404894]
%   data: {"dataType":"symbolic","outputData":{"name":"B_neg1","value":"$$ \\left(\\begin{array}{ccc}\n0.001025 & \\text{4.126e-5} & 0.0001156\\\\\n\\text{4.126e-5} & 0.004228 & \\text{1.44e-6}\\\\\n0.0001156 & \\text{1.44e-6} & 0.008267\n\\end{array}\\right) $$"}}
%---
%[output:8ec3150f]
%   data: {"dataType":"matrix","outputData":{"columns":110,"name":"C_vel","rows":5,"type":"double","value":[["0","0","0","1.0000","0","0","0","0.0083","0","0.0066","0","-0.0046","0","0.0000","0","0.0000","0","-0.0025","0","0.0000","0","-0.0022","0","-0.0025","0","0.0022","0","-0.0000","0","0.0000"]]}}
%---
%[output:0e8c30cc]
%   data: {"dataType":"matrix","outputData":{"columns":110,"name":"C_pos","rows":5,"type":"double","value":[["1.0000","0","0","0","0","0","0.0083","0","0.0066","0","-0.0046","0","0.0000","0","0.0000","0","-0.0025","0","0.0000","0","-0.0022","0","-0.0025","0","0.0022","0","-0.0000","0","0.0000","0"]]}}
%---
%[output:4f5f2c87]
%   data: {"dataType":"matrix","outputData":{"columns":5,"name":"B_p","rows":110,"type":"double","value":[["0","0","0","0","0"]]}}
%---
%[output:60e6b165]
%   data: {"dataType":"matrix","outputData":{"columns":5,"name":"B_p","rows":110,"type":"double","value":[["0","0","0","0","0"]]}}
%---
%[output:61341b1e]
%   data: {"dataType":"matrix","outputData":{"columns":5,"name":"B_n","rows":110,"type":"double","value":[["0","0","0","0","0"]]}}
%---
%[output:7993f976]
%   data: {"dataType":"matrix","outputData":{"columns":5,"name":"B_n","rows":110,"type":"double","value":[["0","0","0","0","0"]]}}
%---
%[output:393a8e5a]
%   data: {"dataType":"matrix","outputData":{"columns":110,"name":"C_vel_comb","rows":3,"type":"double","value":[["0","0","0","1.0000","0","0","0","0.0083","0","0.0066","0","-0.0046","0","0.0000","0","0.0000","0","-0.0025","0","0.0000","0","-0.0022","0","-0.0025","0","0.0022","0","-0.0000","0","0.0000"]]}}
%---
%[output:43103c0d]
%   data: {"dataType":"matrix","outputData":{"columns":110,"name":"C_pos_comb","rows":3,"type":"double","value":[["1.0000","0","0","0","0","0","0.0083","0","0.0066","0","-0.0046","0","0.0000","0","0.0000","0","-0.0025","0","0.0000","0","-0.0022","0","-0.0025","0","0.0022","0","-0.0000","0","0.0000","0"]]}}
%---
%[output:1ef4e237]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"B_pos_comb","rows":110,"type":"double","value":[["0","0","0"]]}}
%---
%[output:8d802426]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"B_pos_comb","rows":110,"type":"double","value":[["0","0","0"]]}}
%---
%[output:106903b1]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"B_neg_comb","rows":110,"type":"double","value":[["0","0","0"]]}}
%---
%[output:37bc266b]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"B_neg_comb","rows":110,"type":"double","value":[["0","0","0"]]}}
%---
%[output:3aa1b2b9]
%   data: {"dataType":"symbolic","outputData":{"name":"S","value":"$$ \\left(\\begin{array}{ccc}\n1 & 0 & 0\\\\\n0 & 1 & 0\\\\\n\\sin \\left(\\frac{1}{10}\\right) & 0 & 1\n\\end{array}\\right) $$"}}
%---
%[output:1b2ff5cc]
%   data: {"dataType":"symbolic","outputData":{"name":"Sn","value":"$$ \\left(\\begin{array}{ccc}\n1 & 0 & 0\\\\\n0 & 1 & 0\\\\\n-\\sin \\left(\\frac{1}{10}\\right) & 0 & 1\n\\end{array}\\right) $$"}}
%---
%[output:58a347c1]
%   data: {"dataType":"textualVariable","outputData":{"header":"logical","name":"p","value":"   1\n"}}
%---
%[output:2520351e]
%   data: {"dataType":"textualVariable","outputData":{"name":"r","value":"1.0000"}}
%---
%[output:5e4d3f57]
%   data: {"dataType":"textualVariable","outputData":{"header":"logical","name":"p","value":"   1\n"}}
%---
%[output:5367a71e]
%   data: {"dataType":"textualVariable","outputData":{"name":"r","value":"1.0000"}}
%---
%[output:0ca3bfce]
%   data: {"dataType":"matrix","outputData":{"columns":18,"exponent":"4","name":"A_use","rows":18,"type":"double","value":[["0","0","0","0.0001","0","0","0","0","0","0","0","0","0","0","0","0","0","0"]]}}
%---
%[output:551df8e1]
%   data: {"dataType":"matrix","outputData":{"columns":5,"name":"B_use","rows":18,"type":"double","value":[["0","0","0","0","0"]]}}
%---
%[output:2fe15d6e]
%   data: {"dataType":"matrix","outputData":{"columns":18,"name":"C_use","rows":5,"type":"double","value":[["0","0","0","1.0000","0","0","0","0.0083","0","0.0066","0","-0.0046","0","0.0000","0","0.0000","0","-0.0025"]]}}
%---
%[output:2aabd6bc]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"-0.1035"}}
%---
%[output:267b182c]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"-0.1035"}}
%---
%[output:7fb21ebc]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 0.330361 seconds.\n","truncated":false}}
%---
