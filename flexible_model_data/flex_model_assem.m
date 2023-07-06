

clear
clc

freqs = extract_freqs;

cd("/home/bholder/bit-matlab-sim/flexible_model_data/RW/")
rw_curl = rw();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/roll_stern/")
roll_stern_curl = roll_stern();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/roll_bow/")
roll_bow_curl = roll_bow();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/pitch_sb/")
pitch_sb_curl = sb_pitch();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/pitch_port/")
pitch_port_curl = port_pitch();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/gyro1_p/")
gyro1_p_curl = gyro1_pitch();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/gyro1_xp/")
gyro1_xp_curl = gyro1_xp();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/gyro1_bore/")
gyro1_bore_curl = gyro1_bore();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/gyro2_p/")
gyro2_p_curl = gyro2_pitch();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/gyro2_xp/")
gyro2_xp_curl = gyro2_xp();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/gyro2_bore/")
gyro2_bore_curl = gyro2_bore();
cd("/home/bholder/bit-matlab-sim/flexible_model_data/")

%% Build the Model
Br = [1 0 0 0 0; 0 1 1 0 0; 0 0 0 1 1];
Be = 0.5*[rw_curl, roll_bow_curl, roll_stern_curl, pitch_port_curl, pitch_sb_curl];
B = [Br;Be];

% Mr = [3526.5, 4483.2, 5748.2]*1000000; %kg*mm^2
Mr = [991, 364, 134]; %kg*mm^2
Me = zeros(1,length(freqs))+1;
M = [Mr, Me];
M = diag(M);

K_vec = zeros(1,3);
K_vec = [K_vec, freqs]*2*pi;%convert from Hz to rad/s
K = diag(K_vec);
K = K*K;

A_hat = [zeros(length(K_vec),length(K_vec)), eye(length(K_vec)); ...
    -M\K zeros(length(K_vec),length(K_vec))];
B_hat = [zeros(length(K_vec),5); M\B];
C_hat = [(M\B)', zeros(size(B'))];
C_enc = [zeros(size(B')), (M\B)'];
C_gyro1 = [zeros(1,3) gyro1_bore_curl', zeros(1, length(gyro1_bore_curl) + 3);...
        zeros(1,3) gyro1_xp_curl', zeros(1, length(gyro1_xp_curl) + 3);...
        zeros(1,3) gyro1_p_curl', zeros(1, length(gyro1_p_curl) + 3)];
C_gyro2 = [zeros(1,3) gyro2_bore_curl', zeros(1, length(gyro2_bore_curl) + 3);...
        zeros(1,3) gyro2_xp_curl', zeros(1, length(gyro2_xp_curl) + 3);...
        zeros(1,3) gyro2_p_curl', zeros(1, length(gyro2_p_curl) + 3)];


%%
K_f = diag(freqs)*2*pi;
K_f = K_f*K_f;

A_f = [zeros(size(K_f)), eye(length(freqs)); ...
    -diag(Me)\K_f zeros(size(K_f))];
B_f = [zeros(length(freqs),5); diag(Me)\Be];

Cf_gyro1 = [gyro1_bore_curl', zeros(1, length(gyro1_bore_curl));...
        gyro1_p_curl', zeros(1, length(gyro1_p_curl));...
        gyro1_xp_curl', zeros(1, length(gyro1_xp_curl))];

Cf_gyro_pos = [zeros(1, length(gyro1_bore_curl)), gyro1_bore_curl';...
        zeros(1, length(gyro1_p_curl)), gyro1_p_curl';...
        zeros(1, length(gyro1_xp_curl)), gyro1_xp_curl'];

Cf_pos_gimbal = [ diag(Me)\Be; zeros(length(freqs),5)]';

Cf_gyro2 = [gyro2_bore_curl', zeros(1, length(gyro2_bore_curl));...
        gyro2_p_curl', zeros(1, length(gyro2_p_curl));...
        gyro2_xp_curl', zeros(1, length(gyro2_xp_curl))];
%% Add Damping
A_mf = zeros(size(A_f));
B_mf = zeros(size(B_f));
zeta = 0.001;
for l = 1:length(freqs)
    w = freqs(l)*2*pi;
    l1 = ((l-1)*2) + 1;
    l2 = 2*l;
    A_mf(l1, l2) = 1;
    A_mf(l2,l1) = -(w^2);
    A_mf(l2, l2) = -2*zeta*w;

    B_mf(l2,:) = Be(l,:);
end

B_mf = B_mf*1000;
B_f = B_f*1000;

%%

sys = ss(A_hat,B_hat,C_hat,0);

save('flex_model',"A_f", "B_f", "C_hat","B_hat","A_hat", "C_enc", "C_hat", "C_gyro1", "C_gyro2", "A_mf", "B_mf");
dlmwrite('Kf.csv', K_f, 'precision', '%.16f')
dlmwrite('Af.csv', A_f, 'precision', '%.16f')
dlmwrite('Bf.csv', B_f, 'precision', '%.16f')
dlmwrite('Amf.csv', A_mf, 'precision', '%.16f')
dlmwrite('Bmf.csv', B_mf, 'precision', '%.16f')
dlmwrite('G1f.csv', Cf_gyro1, 'precision', '%.16f')
dlmwrite('G2f.csv', Cf_gyro2, 'precision', '%.16f')
dlmwrite('G1fp.csv', Cf_gyro_pos, 'precision', '%.16f')
dlmwrite('pos_gimb.csv', Cf_pos_gimbal, 'precision', '%.16f')

a_f_syms = sym(A_f);
f1 = matlabFunction(a_f_syms, 'file', 'a_f_func.m');

b_f_syms = sym(B_f);
f2 = matlabFunction(b_f_syms, 'file', 'b_f_func.m');

a_mf_syms = sym(A_mf);
f1 = matlabFunction(a_mf_syms, 'file', 'a_mf_func.m');

b_mf_syms = sym(B_mf);
f2 = matlabFunction(b_mf_syms, 'file', 'b_mf_func.m');

