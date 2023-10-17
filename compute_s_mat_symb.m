function [s9,s8,s7] = compute_s_mat_symb(theta, z_n)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

s9 = zeros(3,9);
for i = 1:9
    Cn = axis2rot_symb(z_n(:,i), theta(i));
    s9(:,i) = z_n(:,i);
    s9 = Cn*s9; 
    if i == 7
        s7 = s9;
    end
    if i == 8
        s8 = s9;
    end
end


end

