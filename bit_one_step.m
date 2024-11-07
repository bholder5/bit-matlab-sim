function [y_true, y_flex] = bit_one_step(x0, tau_applied, unlock, w_piv, piv_flag,...
    dt, num_steps, tau_max_piv, thet_pit_nom, x_flex0, tau_flex, flexure_flag, sb_flag)
    % Run initialization script
    if sb_flag
        [ndof, g0, r_n1_n, z_n, p_n, m_n, c_n, ...
        i_n, m_w_n,  i_rw, bear_k_cst, bear_c_cst, k_d, b_d, ...
        w_rw_max, w_rw_nom, hs_rw, hs_rw_max, a_flex, b_flex, a_df, b_df] = init_func_sb();
    else 
        [ndof, g0, r_n1_n, z_n, p_n, m_n, c_n, ...
        i_n, m_w_n,  i_rw, bear_k_cst, bear_c_cst, k_d, b_d, ...
        w_rw_max, w_rw_nom, hs_rw, hs_rw_max, a_flex, b_flex, a_df, b_df] = init_func();
    end 
    
    if ~flexure_flag
        k_d(8) = 0;
        k_d(9) = 0;
    end

    %% Setup Simulation
    % initial conditions, state is dtheta; theta
    y_true = x0;
    y_flex = x_flex0;

    % Sim Parameters
    % y_all1 = zeros(18, tf/(dt));
    step = 0;

    sys = @(y_true, tau_applied, dw_piv) bit_propagator(y_true, c_n, z_n, m_n, r_n1_n, m_w_n, p_n, ... 
    k_d, b_d, g0, unlock, hs_rw_max, tau_applied, w_piv, piv_flag, dw_piv, tau_max_piv, thet_pit_nom);
    
    tau_app_flex = tau_applied(7:9);
    
    tau_applied(7) = tau_applied(7) + tau_flex(1);
    tau_applied(8) = tau_applied(8) + tau_flex(2) + tau_flex(3);
    tau_applied(9) = tau_applied(9) + tau_flex(4) + tau_flex(5);
    
    sys_flex = @(y_flex, tau_app_flex, tau_flex) flex_propogate(a_df, b_df, tau_app_flex, tau_flex, y_flex);

%% sim

    for step = 1:num_steps
        %% Propagate the system
        %RK4 solver
        dw_piv = (w_piv - y_true(6))/dt;

        [k1] = sys(y_true, tau_applied, dw_piv) * dt;
        [k2] = sys(y_true + (k1(1:21)/2), tau_applied, dw_piv) * dt;
        [k3] = sys(y_true + (k2(1:21)/2), tau_applied, dw_piv) * dt;
        [k4] = sys(y_true + k3(1:21), tau_applied, dw_piv) * dt;

        temp = ((k1+(2*k2)+(2*k3)+k4)/6);
        tdd = temp(1:21);
        tau_app_flex = temp(22:24)/dt
        y_true = y_true + tdd;   

        th_over = y_true(10:18) > pi;
        th_under = y_true(10:18) < -pi;
        y_true(10:14) = y_true(10:14) -(2*pi*th_over(1:5)) + (2*pi*th_under(1:5));
        y_true(16:18) = y_true(16:18) -(2*pi*th_over(7:9)) + (2*pi*th_under(7:9));
        
%         fprintf('current state:  %0.15f \n  %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n  %0.15f \n %0.15f \n %0.15f \n \n', ...
%             y_true(1), y_true(2), y_true(3), y_true(4), y_true(5), y_true(6),...
%              y_true(7), y_true(8), y_true(9), y_true(10), y_true(11), y_true(12),...
%               y_true(13), y_true(14), y_true(15), y_true(16), y_true(17), y_true(18),...
%                y_true(19), y_true(20), y_true(21));      
        %% Propogate flexible system
        kf1 = sys_flex(y_flex, tau_app_flex, tau_flex) * dt;
        kf2 = sys_flex(y_flex + (kf1/2), tau_app_flex, tau_flex) * dt;
        kf3 = sys_flex(y_flex + (kf2/2), tau_app_flex, tau_flex) * dt;
        kf4 = sys_flex(y_flex + kf3, tau_app_flex, tau_flex) * dt;

        eta_dd = ((kf1+(2*kf2)+(2*kf3)+kf4)/6);
        y_flex = y_flex + eta_dd;  
    end

end

