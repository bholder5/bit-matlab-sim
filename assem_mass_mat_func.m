clear
clc

[ndof, g0, r_n1_n, z_n, p_n, m_n, c_n, ...
    i_n, m_w_n,  i_rw, bear_k_cst, bear_c_cst, k_d, b_d, ...
    w_rw_max, w_rw_nom, hs_rw, hs_rw_max, a_flex, b_flex, a_df, b_df] = init_func();


syms t1 t2 t3 t4 t5 t6 t7 t8 t9

thet = [t1; t2; t3; t4; t5; t6; t7; t8; t9]

M = compute_mass_matrix_symb(thet, z_n, r_n1_n, m_w_n, p_n);

h = matlabFunction(M, 'File', "mass_mat_func_gb.m", "Vars",{thet} );

theta = [1;2;3;4;5;-6;7;8;9];
M = compute_mass_matrix(theta, z_n, r_n1_n, m_w_n, p_n);
M2 = mass_mat_func_gb(theta);

clear
clc

[ndof, g0, r_n1_n, z_n, p_n, m_n, c_n, ...
    i_n, m_w_n,  i_rw, bear_k_cst, bear_c_cst, k_d, b_d, ...
    w_rw_max, w_rw_nom, hs_rw, hs_rw_max, a_flex, b_flex, a_df, b_df] = init_func_sb();


syms t1 t2 t3 t4 t5 t6 t7 t8 t9

thet = [t1; t2; t3; t4; t5; t6; t7; t8; t9]

M = compute_mass_matrix_symb(thet, z_n, r_n1_n, m_w_n, p_n);

h = matlabFunction(M, 'File', "mass_mat_func_sb.m", "Vars",{thet} );

theta = [1;2;3;4;5;-6;7;8;9];
M = compute_mass_matrix(theta, z_n, r_n1_n, m_w_n, p_n);
M2 = mass_mat_func_sb(theta);