cd('/home/bholder/bit-matlab-sim')
addpath('/home/bholder/bit-matlab-sim/Miscellaneous')
addpath('/home/bholder/bit-matlab-sim/flexible_model_data/')
addpath('/home/bholder/bit-matlab-sim/Plant_functions/')
load_sl_glibc_patch
tic
syms t1 t2 t3 t4 t5 t6 t7 t8 t9 real
syms dt1 dt2 dt3 dt4 dt5 dt6 dt7 dt8 dt9 real

zer = sym(0);

thet = [t1 t2 t3 t4 t5 t6 t7 t8 t9]';
thet_m = [zer zer zer zer zer zer zer zer zer]';
d_thet = [dt1; dt2; dt3; dt4; dt5; dt6; dt7; dt8; dt9];

z_n = sym([0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
           0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
           1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]);

s7 = zeros(3,9);
for i = 1:9
    Cn = axis2rot_symb(z_n(:,i), thet(i));
    s7(:,i) = z_n(:,i);
    s7 = Cn*s7;    
end

% s7 = s7';

% s7 = subs(s7, sin(t2), t2);
% s7 = subs(s7, cos(t2), zer);
% s7 = subs(s7, sin(t3), t3);
% s7 = subs(s7, cos(t3), zer);
% s7 = subs(s7, sin(t4), t4);
% s7 = subs(s7, cos(t4), zer);
% s7 = subs(s7, sin(t5), t5);
% s7 = subs(s7, cos(t5), zer);
% s7 = subs(s7, sin(t8), t8);
% s7 = subs(s7, cos(t8), zer)
% % s7 = subs(s7, t3*t4, zer)
% % s7 = subs(s7, t4*t5, zer)
%% 
% 

s = zeros(3,9);
for i = 1:7
    Cn = axis2rot_symb(z_n(:,i), thet(i));
    s(:,i) = z_n(:,i);
    s = Cn*s;    
end

% s = subs(s, sin(t2), t2);
% s = subs(s, cos(t2), zer);
% s = subs(s, sin(t3), t3);
% s = subs(s, cos(t3), zer);
% s = subs(s, sin(t4), t4);
% s = subs(s, cos(t4), zer);
% s = subs(s, sin(t5), t5);
% s = subs(s, cos(t5), zer);

tau_rw = sym('tauRW');
syms hs1 hs2 hs dhs1 dhs2 dhs real

h_s = [zer; zer; hs];
d_hs = [zer; zer; tau_rw];

% z7 = [0.01;0.015;1];
% z7 = z7/sqrt(z7'*z7)
z7 = z_n(:,7);


% d_hs = tau_rw*z7;

r = (s')*d_hs;

R = -(s') * xmat(h_s) * s;
R = simplify(R);
%% 
% 

[ndof, g0, r_n1_n, z_n, p_n, m_n, c_n, ...
    i_n, m_w_n,  i_rw, bear_k_cst, bear_c_cst, k_d, b_d, ...
    w_rw_max, w_rw_nom, hs_rw, hs_rw_max] = init_func_env_test();

M = compute_mass_matrix_symb(thet_m, z_n, r_n1_n, m_w_n, p_n);

% M = subs(M, t1, zer);
% M = subs(M, t2, zer);
% M = subs(M, t3, zer);
% M = subs(M, t4, zer);
% M = subs(M, t5, zer);
% M = subs(M, t6, zer);
% M = subs(M, {t1, t2, t3, t4, t5, t6}, {zer, zer, zer, zer, zer, zer});

% toc
Mi = inv(M);



Pot = compute_potential_energy_term_symb(thet, c_n, z_n, m_n, r_n1_n, g0);

% theta_spring = theta;
% theta_spring(9) = theta(9) - thet_pit_nom;
% 
spring = k_d.*thet;

% damp = b_d.*dtheta;
% 
% %place holder
% [R,r, d_hs] = RW_terms(theta, dtheta, z_n, hs, tau_rw, hs_rw_max);
% 
% %calculate joint torques from gravity elasticity and damnping according
% %to eq 3.37
% torques = tau_applied - (Pot + spring + damp + R + r);
syms tau_p tau_r

tau = sym(zeros(9,1));
tau(8) = tau_r;
tau(9) = tau_p;
%%
mod = Mi*(tau-(R*d_thet)-r-spring-Pot);
% mod = simplify(mod);

A_pend = [diff(mod, t1), diff(mod, t2), diff(mod, t3), ...
    diff(mod, t4), diff(mod, t5), diff(mod, t6), diff(mod, t7), ...
    diff(mod, t8), diff(mod, t9), diff(mod, dt1), diff(mod, dt2), ...
     diff(mod, dt3),  diff(mod, dt4), diff(mod, dt5),  diff(mod, dt6) ...
    diff(mod, dt7),  diff(mod, dt8), diff(mod, dt9), ...
    diff(mod, hs), diff(mod, dhs)];


hs_vals = z7*9*2*pi;
% A_pend = simplify(A_pend);
% A_pend = subs(A_pend, t2, zer);
% A_pend = subs(A_pend, t3, zer);
% A_pend = subs(A_pend, t4, zer);
% A_pend = subs(A_pend, t5, zer);
% A_pend = subs(A_pend, t6, zer);
% A_pend = subs(A_pend, t7, zer);
% A_pend = subs(A_pend, t1, zer);
% A_pend = subs(A_pend, dt1, zer);
% A_pend = subs(A_pend, dt2, zer);
% A_pend = subs(A_pend, dt3, zer);
% A_pend = subs(A_pend, dt4, zer);
% A_pend = subs(A_pend, dt5, zer);
% A_pend = subs(A_pend, dt6, zer);
% A_pend = subs(A_pend, dt7, zer);
% A_pend = subs(A_pend, dt8, zer);
% A_pend = subs(A_pend, dt9, zer);
% A_pend = subs(A_pend, hs, hs_vals(3));
% A_pend = subs(A_pend, tau_rw, zer);

A_pend = subs(A_pend, ...
    {t1, t2, t3, t4, t5, t6, t7, dt1, dt2, dt3, dt4, dt5, dt6, dt7, dt8, dt9, hs, tau_rw}, ...
    {zer, zer, zer, zer, zer, zer, zer, zer, zer, zer, zer, zer, zer, zer, zer, zer, hs_vals(3), zer});

% A_pend = subs(A_pend, tau_r, zer);
% A_pend = subs(A_pend, tau_p, zer);



A = sym(zeros(20,20));
A(1:9,10:18) = eye(9);
A(10:18,1:20) = eval(A_pend);
% B_pend = [diff(mod, tau_rw)];


%%
roll_torque = 0.1*k_d(8);

% Ap = subs(A, t7, zer);
% Ap = subs(Ap, t8, zer);
% Ap = subs(Ap, t9, zer);
% Ap = subs(Ap, tau_p, zer);
% Ap = subs(Ap, tau_rw, zer);
% Ap = subs(Ap, tau_r, zer);
Ap = subs(A, {t7, t8, t9, tau_p, tau_rw, tau_r}, {zer, zer, zer, zer, zer, zer});

pl = eig(eval(Ap))/(2*pi)
toc
 
% 
%