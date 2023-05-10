function [omega] = compute_angular_velocity_yaw_C(x, z_n)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
theta = x(10:18);
dtheta = x(1:7);

s7 = zeros(3,7);
for i = 1:7
    Cn = axis2rot(z_n(:,i), theta(i));
    s7(:,i) = z_n(:,i);
    s7 = Cn*s7;    
end

omega = s7 * dtheta;

end

