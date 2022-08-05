function [C] = compute_rotation_mat_C(z_n, theta)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
C = (eye(3));
for i = 1:9
    C = axis2rot(z_n(:,i), theta(i)) * C;
end
C = C';
end

