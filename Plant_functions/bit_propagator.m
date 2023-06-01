function [Xdot] = bit_propagator(X, c_n, z_n, m_n, r_n1_n, m_w_n, p_n, ... 
    k_d, b_d, g0, unlock, hs_rw_max, tau_applied, w_piv, piv_flag,...
    dw_piv, tau_max_piv, thet_pit_nom)
    %split the state

    theta = X(10:18);
    dtheta = X(1:9);
    hs = X(19:21);
    
    %extract RW torque.
    tau_rw = tau_applied(7);
    tau_applied(7) = 0;  

    % set pivot speed in dtheta...
    if piv_flag == true
        dtheta(6) = w_piv;
    end
    
    %%
    Pot = compute_potential_energy_term(theta, c_n, z_n, m_n, r_n1_n, g0);

    theta_spring = theta;
    theta_spring(9) = theta(9) - thet_pit_nom;
    
    spring = k_d.*theta_spring;
    damp = b_d.*dtheta;
    
    %place holder
    [R,r, d_hs] = RW_terms(theta, dtheta, z_n, hs, tau_rw, hs_rw_max);
       
    %calculate joint torques from gravity elasticity and damnping according
    %to eq 3.37
    torques = tau_applied - (Pot + spring + damp + R + r);
 
    % M = compute_mass_matrix(theta, z_n, r_n1_n, m_w_n, p_n);

    % M = mass_mat_func(theta);
    M = mass_mat_func_gb(theta);

    M_decomp = chol(M);

    ddtheta = M_decomp\((M_decomp')\torques);
    ddtheta = ddtheta.*unlock;

    if piv_flag == true
        prop_err = 10;
        int_err = 0;
        kp = 1;
        ki = 0.5;
        prop_err = dw_piv - ddtheta(6);
        int_err = int_err + prop_err;
        tau_piv = torques(6);

        while abs(prop_err) > 1e-9
            
            tau_piv = tau_piv + ((kp*prop_err) + (ki*int_err));
            if abs(tau_piv) > tau_max_piv
                tau_piv = sign(tau_piv) * tau_max_piv;
                torques(6) = tau_piv;

                ddtheta = M_decomp\((M_decomp')\torques);
                break
            end
            torques(6) = tau_piv;

            ddtheta = M_decomp\((M_decomp')\torques);
            prop_err = dw_piv - ddtheta(6);
            int_err = int_err + prop_err;
        end
    end

    tau_gond = M(7:9,7:9) * ddtheta(7:9);
    % tau_gond(1) = tau_rw

    Xdot = [ddtheta; dtheta; d_hs; tau_gond];
    
end