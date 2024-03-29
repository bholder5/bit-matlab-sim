function [R, r, d_hs] = RW_terms(theta, dtheta, z_n, hs, ...
    tau_rw, hs_rw_max)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%calculate the mapping matrix from dtheta to omega
s7 = zeros(3,9);
for i = 1:7
    Cn = axis2rot(z_n(:,i), theta(i));
    s7(:,i) = z_n(:,i);
    s7 = Cn*s7;    
end


d_hs = tau_rw*z_n(:,7);

if hs(3) >= hs_rw_max
    if d_hs(3) > 0
        d_hs(3) = 0;
    end
elseif hs(3) <= -hs_rw_max
    if d_hs(3) < 0
        d_hs(3) = 0;
    end
end

r = (s7')*d_hs;

R = -(s7') * xmat(hs) * s7 * dtheta;
% rw_g2 = 2.353962297635081;
% rw_g1 = 0.034;
% w_piv = rw_g1*((-hs(3)/i_rw(3,3))-w_rw_nom) - (rw_g2*tau_rw);

end

