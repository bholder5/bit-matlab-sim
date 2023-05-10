function [omega] = compute_angular_velocity_roll_C(x, z_n)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
theta = x(10:18);
dtheta = x(1:8);

s8 = zeros(3,8);
for i = 1:8
    Cn = axis2rot(z_n(:,i), theta(i));
    s8(:,i) = z_n(:,i);
    s8 = Cn*s8;    
end

omega = s8 * dtheta;

end

