function Xdot = bit_propagator(X, c_n, z_n, m_n, r_n1_n, m_w_n, p_n, ... 
    k_d, b_d, g0, i_rw, unlock, hs_rw_max, rw_g1, rw_g2, w_rw_nom, tau_applied)
    %split the state
    theta = X(10:18);
    dtheta = X(1:9);
    hs = X(19:21);
    
    %extract RW torque.
    tau_rw = tau_applied(7);
    tau_applied(7) = 0;
        
%     tau_applied = zeros(9,1);
%    tau_rw = tau_applied(7);
    
    %%
    Pot = compute_potential_energy_term(theta, c_n, z_n, m_n, r_n1_n, g0);
    
    spring = k_d.*theta;
    damp = b_d.*dtheta;
    
    %place holder
    [R,r, d_hs, w_piv] = RW_terms(theta, dtheta, z_n,i_rw, hs, tau_rw, hs_rw_max, ...
        rw_g1, rw_g2, w_rw_nom);
    
    % calculate the ddtheta resulting from desired 
    
    %calculate joint torques from gravity elasticity and damnping according
    %to eq 3.37
    torques = tau_applied - (Pot + spring + damp + R + r);
    
    M = compute_mass_matrix(theta, z_n, r_n1_n, m_w_n, p_n);
%     Mass = mass_mat_func(theta);
    
    M_decomp = chol(M);
    ddtheta = M_decomp\(M_decomp'\torques);
    
    ddtheta = ddtheta.*unlock;
    
    % set pivot speed in dtheta...
    dtheta(6) = w_piv;
    Xdot = [ddtheta; dtheta; d_hs;];
    
    
end