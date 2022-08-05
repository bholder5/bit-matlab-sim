function [omega] = compute_angular_velocity_C(x, z_n)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
theta = x(10:18);
dtheta = x(1:9);

s9 = zeros(3,9);
for i = 1:9
    Cn = axis2rot(z_n(:,i), theta(i));
    s9(:,i) = z_n(:,i);
    s9 = Cn*s9;    
end

omega = s9 * dtheta;

end

