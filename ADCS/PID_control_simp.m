function [tau_joint] = PID_control_simp(error, omega,theta, dtheta,z_n, sum, kp_f, kd_f, ki_f, ...
    kp_rw, kd_rw, ki_rw, tau_f_max)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%     phi = (s79'*s79);
    phi = [1 0 0; 0 1 sin(theta(2)); 0 sin(theta(2)) 1];
       
       %error is in 123 format but dtheta is in 312
       % i think this is actually wrong because we want to actuate on 
       % the overall movement of the gondola, not on the movement of the
       % last 3 frames (this will stop us from compensating for movement
       % from upper joints aka balloon
    dthet_err = -[dtheta(2); dtheta(3); dtheta(1)];
    tau_f  = (diag(kp_f) * phi * error) + ...
             (diag(kd_f) * phi * dthet_err) + ...
             (diag(ki_f) * phi * sum);
    
    for k = 1:2
        if abs(tau_f(k)) > tau_f_max
            tau_f(k) = sign(tau_f(k))*tau_f_max;
        end
    end
    
    if abs(tau_f(3)) > 80
        tau_f(3) = sign(tau_f(3))*80;
    end

    tau_joint = zeros(9,1);
    tau_joint(7) = -tau_f(3);
    tau_joint(8) = tau_f(1);
    tau_joint(9) = tau_f(2);
    
end

