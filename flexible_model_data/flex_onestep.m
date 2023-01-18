function [eta_true] = flex_onestep(eta0, tau_applied, ...
    dt, num_steps, A_hat, B_hat)
    %% Setup Simulation
    % initial conditions, state is dtheta; theta
    eta_true = eta0;
    
    % Sim Parameters
    step = 0;
    
    %took out t from (t, y_true) for rk4
    sys = @(x_true, tau_applied) ((A_hat * x_true) + (B_hat * tau_applied))
    
    %% Sim
    for step = 1:num_steps
            %% Propagate the system
            %RK4 solver
        
        %% Propagate the system 
        %RK4 solver
    
        k1 = sys(eta_true, tau_applied) * dt;
        k2 = sys(eta_true + (k1/2), tau_applied) * dt;
        k3 = sys(eta_true + (k2/2), tau_applied) * dt;
        k4 = sys(eta_true + k3, tau_applied) * dt;
    % 
        tdd = ((k1+(2*k2)+(2*k3)+k4)/6);
        eta_true = eta_true + tdd   
    
    end
end
