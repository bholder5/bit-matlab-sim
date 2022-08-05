function [C] = compute_rotation_mat_sym(z_n, theta)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
C = sym(eye(3));
for i = 1:9
    C = axis2rotsymb(z_n(:,i), theta(i)) * C;
end

end

