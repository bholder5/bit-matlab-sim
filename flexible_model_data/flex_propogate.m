function [eta_dot] = flex_propogate(a_flex, b_flex, tau_applied, tau_flex, x0_flex)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    tau = tau_flex;

    tau_yaw = tau_applied(1) - tau(1);
    tau_roll = tau_applied(2) - (tau(2) + tau(3));
    tau_pitch = tau_applied(3) - (tau(4) + tau(5));
    % 
    
    tau(1) = tau(1) + tau_yaw;
    tau(2) = tau(2) + (tau_roll/2);
    tau(3) = tau(3) + (tau_roll/2);
    tau(4) = tau(4) + (tau_pitch/2);
    tau(5) = tau(5) + (tau_pitch/2);
    

    eta_dot = (a_flex * x0_flex) + (b_flex * tau);
end