function [tau_joint] = PID_control(error, omega,theta, dtheta,z_n, sum, kp_f, kd_f, ki_f, ...
    kp_rw, kd_rw, ki_rw, tau_f_max)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    s79 = zeros(3,3);
    for i = 1:3
        Cn = axis2rot(z_n(:,i+6), theta(i));
        s79(:,i) = z_n(:,i+6);
        s79 = Cn*s79;    
    end
    
    %verify this is the same ss 5.41-b.
%     phi = (s79'*s79);
    phi = [1 0 0; 0 1 sin(theta(2)); 0 sin(theta(2)) 1];
       error;
       sum;
       
       %error is in 123 format but dtheta is in 312
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

