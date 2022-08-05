function Xdot = bit_propagator_mfunc(X, c_n, z_n, m_n, r_n1_n, m_w_n, p_n, ... 
    k_d, b_d, g0, unlock, hs_rw_max, tau_applied, w_piv, piv_flag)
    %split the state
    theta = (X(10:18));
    dtheta = X(1:9);
    hs = X(19:21);
    

    Pot = compute_potential_energy_term(theta, c_n, z_n, m_n, r_n1_n, g0);
%     M = compute_mass_matrix(theta, z_n, r_n1_n, m_w_n, p_n);
    M = mass_mat_func(theta);
    
    spring = k_d.*theta;
    damp = b_d.*dtheta;
    
    %place holder
    control_torque = 0;
    
    %calculate joint torques from gravity elasticity and damnping according
    %to eq 3.37
    torques = control_torque - (Pot + spring + damp);
    
    M_decomp = chol(M);
    ddtheta = M_decomp\(M_decomp'\torques);
    
    ddtheta = ddtheta.*unlock;

    Xdot = [ddtheta; dtheta];
    
    
end